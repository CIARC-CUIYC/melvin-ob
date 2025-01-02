use crate::flight_control::{
    camera_state::CameraAngle,
    common::{bitmap::Bitmap, img_buffer::Buffer, vec2d::Vec2D},
    flight_computer::FlightComputer,
};
use crate::http_handler::{
    http_client::HTTPClient, http_request::request_common::NoBodyHTTPRequestType,
    http_request::shoot_image_get::ShootImageRequest,
};
use futures::{StreamExt, TryStreamExt};
use image::{imageops::Lanczos3, ImageReader};
use std::io::Cursor;
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncWriteExt},
};
use tokio_util::io::StreamReader;

/// Represents a controller that manages camera operations, including saving images to disk
/// or a buffer, and updating regions within a bitmap representation.
///
/// # Fields
/// - `bitmap`: A `Bitmap` for storing and managing the region map.
/// - `buffer`: A `Buffer` for storing image data in memory.
#[allow(clippy::unsafe_derive_deserialize)]
#[derive(serde::Serialize, serde::Deserialize)]
pub struct CameraController {
    /// The bitmap which stores metainformation.
    bitmap: Bitmap,
    /// The buffer storing actual RGB data.
    buffer: Buffer,
}

impl CameraController {
    /// Creates a new `CameraController` instance with default map-sized bitmap and buffer.
    ///
    /// # Returns
    /// A newly initialized `CameraController`.
    pub fn new() -> Self {
        let bitmap = Bitmap::from_map_size();
        let buffer = Buffer::from_map_size();
        Self { bitmap, buffer }
    }

    /// Creates a `CameraController` instance by deserializing its content from a binary file.
    ///
    /// # Arguments
    /// - `path`: The path to the binary file to be read.
    ///
    /// # Errors
    /// - Returns an error if the file cannot be opened, read, or its contents fail deserialization.
    ///
    /// # Returns
    /// An instance of `CameraController` loaded from the specified file.
    pub async fn from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut file = File::open(path).await?;
        let mut file_buffer = Vec::new();
        file.read_to_end(&mut file_buffer).await?;

        let camera = bincode::deserialize(&file_buffer)?;
        Ok(camera)
    }

    /// Provides an immutable reference to the bitmap.
    ///
    /// # Returns
    /// A reference to the `Bitmap` of the controller.
    pub fn bitmap_ref(&self) -> &Bitmap { &self.bitmap }

    /// Provides an immutable reference to the buffer.
    ///
    /// # Returns
    /// A reference to the `Buffer` of the controller.
    pub fn buffer_ref(&self) -> &Buffer { &self.buffer }

    /// Fetches an image from MELVIN DRS and writes it to a file on disk.
    ///
    /// # Arguments
    /// - `http_client`: An HTTP client instance for sending the request.
    /// - `file_path`: The destination path for saving the retrieved image.
    ///
    /// # Errors
    /// - Returns an error if the HTTP request fails or the file cannot be written to disk.
    pub async fn shoot_image_to_disk(
        http_client: &HTTPClient,
        file_path: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut file = File::create(file_path).await?;
        let mut response_stream = ShootImageRequest {}.send_request(http_client).await?;
        while let Some(Ok(chunk)) = response_stream.next().await {
            file.write_all(&chunk).await?;
        }

        file.flush().await?;

        //TODO: Coordinate parsing?
        // self._region(x, y);

        Ok(())
    }

    /// Fetches an image from MELVIN DRS and stores it in the buffer after processing.
    ///
    /// # Arguments
    /// - `http_client`: The HTTP client used to send the image request.
    /// - `controller`: The flight computer managing flight data.
    /// - `angle`: Camera angle information for determining the image size.
    ///
    /// # Errors
    /// - Returns an error if the HTTP request fails, data cannot be downloaded,
    ///   or any subsequent decoding/parsing operations fail.
    #[allow(clippy::cast_possible_truncation)]
    pub async fn shoot_image_to_buffer(
        &mut self,
        http_client: &HTTPClient,
        controller: &mut FlightComputer<'_>,
        angle: CameraAngle,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let ((), collected_png) = tokio::join!(
            controller.update_observation(),
            self.fetch_image_data(http_client)
        );
        let collected_png = collected_png?;
        let decoded_image = Self::decode_and_resize_png(collected_png, angle)?;
        let position = controller.current_pos();
        let pos_x = position.x().round() as i32;
        let pos_y = position.y().round() as i32;
        let angle_const = i32::from(angle.get_square_side_length() / 2);

        for (i, row) in (pos_x - angle_const..pos_x + angle_const).enumerate() {
            for (j, col) in (pos_y - angle_const..pos_y + angle_const).enumerate() {
                let row_u32 = u32::try_from(row).expect("[FATAL] Conversion to u32 failed!");
                let col_u32 = u32::try_from(col).expect("[FATAL] Conversion to u32 failed!");
                let mut coord2d = Vec2D::new(row_u32, col_u32);
                coord2d.wrap_around_map();
                let pixel = decoded_image.get_pixel(i as u32, j as u32);
                self.buffer.save_pixel(coord2d, pixel.0);
            }
        }
        self.bitmap
            .set_region(Vec2D::new(position.x(), position.y()), angle, true);
        Ok(())
    }

    /// Fetches raw image data from the camera via HTTP and returns it as a byte vector.
    ///
    /// # Arguments
    /// - `http_client`: The HTTP client used to send requests.
    ///
    /// # Errors
    /// - Returns an error if the HTTP stream fails or cannot be read properly.
    ///
    /// # Returns
    /// A vector of bytes representing the raw PNG image data.
    async fn fetch_image_data(
        &mut self,
        http_client: &HTTPClient,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let stream = ShootImageRequest {}.send_request(http_client).await?;
        let mapped_stream = stream.map(|result| {
            result.map_err(|err| std::io::Error::new(std::io::ErrorKind::Other, err))
        });
        let mut reader = StreamReader::new(mapped_stream);

        let mut collected_png: Vec<u8> = Vec::new();
        reader.read_to_end(&mut collected_png).await?;
        Ok(collected_png)
    }

    /// Decodes raw PNG data into an RGB image and resizes it based on the camera angle.
    ///
    /// # Arguments
    /// - `collected_png`: A slice containing raw PNG data.
    /// - `angle`: The camera angle used to calculate the target image size.
    ///
    /// # Errors
    /// - Returns an error if the decoding or resizing operation fails.
    ///
    /// # Returns
    /// A resized RGB image.
    fn decode_and_resize_png(
        png: Vec<u8>,
        angle: CameraAngle,
    ) -> Result<image::RgbImage, Box<dyn std::error::Error>> {
        let mut decoded_image = ImageReader::new(Cursor::new(png))
            .with_guessed_format()?
            .decode()?
            .to_rgb8();
        let resized_unit_length = u32::from(angle.get_square_side_length());
        if resized_unit_length > decoded_image.height() {
            decoded_image = image::imageops::resize(
                &decoded_image,
                resized_unit_length,
                resized_unit_length,
                Lanczos3,
            );
        }
        Ok(decoded_image)
    }

    /// Exports the current state of the `CameraController` to a binary file.
    ///
    /// # Arguments
    /// - `path`: The file path where the data will be exported.
    ///
    /// # Errors
    /// - Returns an error if the file could not be created, written to, or flushed.
    pub async fn export_bin(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let encoded = bincode::serialize(&self)?;
        let mut bin_file = File::create(path).await?;
        bin_file.write_all(&encoded).await?;
        bin_file.flush().await?;
        Ok(())
    }
}
