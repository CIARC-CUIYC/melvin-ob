use crate::flight_control::{
    camera_state::CameraAngle,
    common::{bitmap::Bitmap, vec2d::Vec2D},
    flight_computer::FlightComputer,
};
use crate::http_handler::{
    http_client::HTTPClient, http_request::request_common::NoBodyHTTPRequestType,
    http_request::shoot_image_get::ShootImageRequest,
};
use bitvec::boxed::BitBox;
use futures::StreamExt;
use image::buffer::ConvertBuffer;
use image::codecs::png::{CompressionType, FilterType, PngDecoder, PngEncoder};
use image::{
    imageops::Lanczos3, DynamicImage, GenericImage, GenericImageView, ImageBuffer, ImageDecoder,
    ImageReader, Pixel, Rgb, RgbImage, Rgba, RgbaImage,
};
use num_traits::Float;
use std::borrow::Borrow;
use std::io::Cursor;
use std::ops::{Deref, DerefMut};
use tokio::sync::RwLock;
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncWriteExt},
};

/// Represents a controller that manages camera operations, including saving images to disk
/// or a buffer, and updating regions within a bitmap representation.
///
/// # Fields
/// - `bitmap`: A `Bitmap` for storing and managing the region map.
/// - `buffer`: A `Buffer` for storing image data in memory.
#[derive(serde::Serialize, serde::Deserialize)]
#[allow(clippy::unsafe_derive_deserialize)]
pub struct MapImageStorage<T: Borrow<Vec<u8>>, B: Borrow<Bitmap>> {
    coverage: B,
    fullsize_buffer: T,
}

pub struct MapImageThumbnail {
    buffer: RgbaImage,
}

impl MapImageThumbnail {
    pub const THUMBNAIL_SCALE_FACTOR: u32 = 25;

    pub fn thumbnail_size() -> Vec2D<u32> {
        Vec2D::map_size() / Self::THUMBNAIL_SCALE_FACTOR
    }

    fn new() -> Self {
        Self {
            buffer: ImageBuffer::new(Self::thumbnail_size().x(), Self::thumbnail_size().y()),
        }
    }

    fn from(image: &MapImage) -> Self {
        let thumbnail_buffer = image::imageops::thumbnail(
            image,
            Self::thumbnail_size().x(),
            Self::thumbnail_size().y(),
        );
        Self {
            buffer: thumbnail_buffer,
        }
    }

    pub fn view(
        &self,
        offset: Vec2D<u32>,
        size: Vec2D<u32>,
    ) -> SubBuffer<&ImageBuffer<Rgba<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &self.buffer,
            buffer_size: Self::thumbnail_size(),
            offset,
            size,
        }
    }

    pub fn mut_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Rgba<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &mut self.buffer,
            buffer_size: Self::thumbnail_size(),
            offset,
            size: Self::thumbnail_size(),
        }
    }
}

pub struct MapImage {
    coverage: Bitmap,
    fullsize_buffer: RgbImage,
}

impl MapImage {
    fn new() -> Self {
        Self {
            coverage: Bitmap::from_map_size(),
            fullsize_buffer: ImageBuffer::new(Vec2D::map_size().x(), Vec2D::map_size().y()),
        }
    }

    pub async fn from_bin_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut file = File::open(path).await?;
        let mut file_buffer = Vec::new();
        file.read_to_end(&mut file_buffer).await?;

        let image: MapImageStorage<Vec<u8>, Bitmap> = bincode::deserialize(&file_buffer)?;
        let fullsize_buffer = ImageBuffer::from_raw(
            Vec2D::map_size().x(),
            Vec2D::map_size().y(),
            image.fullsize_buffer,
        )
            .unwrap();

        let map_image = Self {
            coverage: image.coverage,
            fullsize_buffer,
        };

        Ok(map_image)
    }

    /// Exports the current state of the `CameraController` to a binary file.
    ///
    /// # Arguments
    /// - `path`: The file path where the data will be exported.
    ///
    /// # Errors
    /// - Returns an error if the file could not be created, written to, or flushed.
    pub async fn export_bin_file(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let image_storage = MapImageStorage {
            coverage: &self.coverage,
            fullsize_buffer: self.fullsize_buffer.as_raw(),
        };
        let encoded = bincode::serialize(&image_storage)?;
        let mut bin_file = File::create(path).await?;
        bin_file.write_all(&encoded).await?;
        bin_file.flush().await?;
        Ok(())
    }

    pub fn view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<&MapImage> {
        SubBuffer {
            buffer: &self,
            buffer_size: Vec2D::map_size(),
            offset,
            size,
        }
    }

    pub fn mut_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Rgb<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &mut self.fullsize_buffer,
            buffer_size: Vec2D::map_size(),
            offset,
            size: Vec2D::map_size(),
        }
    }
}

impl GenericImageView for MapImage {
    type Pixel = Rgba<u8>;

    fn dimensions(&self) -> (u32, u32) {
        self.fullsize_buffer.dimensions()
    }

    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel {
        if self.coverage.is_set(x, y) {
            let pixel = self.fullsize_buffer.get_pixel(x, y).0;
            Rgba([pixel[0], pixel[1], pixel[2], 0xFF])
        } else {
            Rgba([0, 0, 0, 0])
        }
    }
}

pub struct SubBuffer<T> {
    pub(crate) buffer: T,
    pub(crate) buffer_size: Vec2D<u32>,
    pub(crate) offset: Vec2D<u32>,
    pub(crate) size: Vec2D<u32>,
}

impl<T, C> GenericImageView for SubBuffer<T>
where
    T: Deref<Target=C>,
    C: GenericImageView + Sized,
{
    type Pixel = C::Pixel;

    fn dimensions(&self) -> (u32, u32) {
        (self.size.x(), self.size.y())
    }

    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel {
        let x = (x + self.offset.x()) % self.buffer_size.x();
        let y = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.get_pixel(x, y)
    }
}

impl<T> GenericImage for SubBuffer<T>
where
    T: DerefMut,
    T::Target: GenericImage + Sized,
{
    fn get_pixel_mut(&mut self, x: u32, y: u32) -> &mut Self::Pixel {
        let x = (x + self.offset.x()) % self.buffer_size.x();
        let y = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.get_pixel_mut(x, y)
    }

    fn put_pixel(&mut self, x: u32, y: u32, pixel: Self::Pixel) {
        let x = (x + self.offset.x()) % self.buffer_size.x();
        let y = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.put_pixel(x, y, pixel)
    }

    fn blend_pixel(&mut self, x: u32, y: u32, pixel: Self::Pixel) {
        let x = (x + self.offset.x()) % self.buffer_size.x();
        let y = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.blend_pixel(x, y, pixel)
    }
}

pub struct CameraController {
    map_image: RwLock<MapImage>,
    map_image_thumbnail: RwLock<MapImageThumbnail>,
}

impl CameraController {
    /// Creates a new `CameraController` instance with default map-sized bitmap and buffer.
    ///
    /// # Returns
    /// A newly initialized `CameraController`.
    pub fn new() -> Self {
        Self {
            map_image: RwLock::new(MapImage::new()),
            map_image_thumbnail: RwLock::new(MapImageThumbnail::new()),
        }
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
        let map_image = MapImage::from_bin_file(path).await?;
        let map_image_thumbnail = MapImageThumbnail::from(&map_image);

        Ok(Self {
            map_image: RwLock::new(map_image),
            map_image_thumbnail: RwLock::new(map_image_thumbnail),
        })
    }

    pub async fn clone_coverage_bitmap(&self) -> BitBox {
        self.map_image.read().await.coverage.data.clone()
    }

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

    fn score_offset(
        decoded_image: &RgbImage,
        base: &MapImage,
        offset_x: u32,
        offset_y: u32,
    ) -> Vec2D<u32> {
        let mut best_score = i32::MIN;
        let mut best_offset = Vec2D::new(offset_x, offset_y);
        for additional_offset_x in -2..=2 {
            for additional_offset_y in -2..=2 {
                let offset: Vec2D<u32> = Vec2D::new(
                    offset_x as i32 + additional_offset_x,
                    offset_y as i32 + additional_offset_y,
                )
                    .wrap_around_map()
                    .cast();
                let map_image_view = base.view(
                    offset,
                    Vec2D::new(decoded_image.width(), decoded_image.height()),
                );
                let score: i32 = map_image_view
                    .pixels()
                    .zip(decoded_image.pixels())
                    .map(|((_, _, existing_pixel), new_pixel)| {
                        if existing_pixel.0[3] == 0 {
                            0
                        } else if existing_pixel.to_rgb() == new_pixel.to_rgb() {
                            0
                        } else {
                            -1
                        }
                    })
                    .sum();

                let score = score - additional_offset_x.abs() - additional_offset_y.abs();
                if score > best_score {
                    best_offset = offset;
                    best_score = score;
                }
            }
        }
        best_offset
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
        &self,
        http_client: &HTTPClient,
        controller: &mut FlightComputer<'_>,
        angle: CameraAngle,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let ((), collected_png) = tokio::join!(
            controller.update_observation(),
            Self::fetch_image_data(http_client)
        );
        let decoded_image = Self::decode_png_data(
            &collected_png.expect("[ERROR] PNG couldn't be unwrapped"),
            angle,
        )?;
        let center_position = controller.current_pos();
        let size = angle.get_square_side_length() as u32;
        let offset: Vec2D<u32> = Self::center_pos_to_offset(center_position, angle);

        {
            let mut map_image = self.map_image.write().await;
            let offset = Self::score_offset(&decoded_image, &map_image, offset.x(), offset.y());
            let mut map_image_view = map_image.mut_view(offset);

            map_image_view.copy_from(&decoded_image, 0, 0).unwrap();
            map_image.coverage.set_region(offset, size, true);
        }
        self.update_thumbnail(offset, size).await?;
        Ok(())
    }

    async fn update_thumbnail(
        &self,
        offset: Vec2D<u32>,
        size: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let thumbnail_conversion_offset: Vec2D<u32> = Vec2D::new(
            offset.x() as i32 - MapImageThumbnail::THUMBNAIL_SCALE_FACTOR as i32 * 2,
            offset.y() as i32 - MapImageThumbnail::THUMBNAIL_SCALE_FACTOR as i32 * 2,
        )
            .wrap_around_map()
            .cast();

        let thumbnail_conversion_offset = Vec2D::new(
            (thumbnail_conversion_offset.x() / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR)
                * MapImageThumbnail::THUMBNAIL_SCALE_FACTOR,
            (thumbnail_conversion_offset.y() / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR)
                * MapImageThumbnail::THUMBNAIL_SCALE_FACTOR,
        );
        let thumbnail_conversion_size = size + MapImageThumbnail::THUMBNAIL_SCALE_FACTOR * 4;
        let map_image = self.map_image.read().await;
        let map_image_view = map_image.view(
            thumbnail_conversion_offset,
            Vec2D::new(thumbnail_conversion_size, thumbnail_conversion_size),
        );

        let resized_image = image::imageops::thumbnail(
            &map_image_view,
            thumbnail_conversion_size / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR,
            thumbnail_conversion_size / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR,
        );
        let mut map_image_thumbnail = self.map_image_thumbnail.write().await;
        map_image_thumbnail
            .mut_view(thumbnail_conversion_offset / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR)
            .copy_from(&resized_image, 0, 0)
            .unwrap();

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
        http_client: &HTTPClient,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let response_stream = ShootImageRequest {}.send_request(http_client).await?;

        let mut collected_png: Vec<u8> = Vec::new();
        futures::pin_mut!(response_stream);

        while let Some(Ok(chunk_result)) = response_stream.next().await {
            collected_png.extend_from_slice(&chunk_result[..]);
        }

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
    fn decode_png_data(
        collected_png: &[u8],
        angle: CameraAngle,
    ) -> Result<RgbImage, Box<dyn std::error::Error>> {
        let decoded_image = ImageReader::new(Cursor::new(collected_png))
            .with_guessed_format()?
            .decode()?
            .to_rgb8();
        let resized_unit_length = angle.get_square_side_length();

        let resized_image = image::imageops::resize(
            &decoded_image,
            u32::from(resized_unit_length),
            u32::from(resized_unit_length),
            Lanczos3,
        );

        Ok(resized_image)
    }

    pub(crate) async fn export_full_thumbnail_png(
        &self,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let map_image_thumbnail = self.map_image_thumbnail.read().await;
        let mut writer = Cursor::new(Vec::<u8>::new());
        map_image_thumbnail
            .buffer
            .write_with_encoder(PngEncoder::new(&mut writer))?;
        Ok(writer.into_inner())
    }

    pub(crate) async fn export_thumbnail_png(
        &self,
        offset: Vec2D<u32>,
        size: u32,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let map_image_thumbnail = self.map_image_thumbnail.read().await;
        let thumbnail = map_image_thumbnail.view(
            Vec2D::new(
                offset.x() / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR,
                offset.y() / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR,
            ),
            Vec2D::new(
                size / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR,
                size / MapImageThumbnail::THUMBNAIL_SCALE_FACTOR,
            ),
        );

        let mut thumbnail_image = RgbaImage::new(thumbnail.width(), thumbnail.width());
        thumbnail_image.copy_from(&thumbnail, 0, 0).unwrap();
        let mut writer = Cursor::new(Vec::<u8>::new());
        //let thumbnail_image: RgbImage = thumbnail_image.convert();
        thumbnail_image.write_with_encoder(PngEncoder::new(&mut writer))?;

        Ok(writer.into_inner())
    }

    /// Exports the current state of the `CameraController` to a binary file.
    ///
    /// # Arguments
    /// - `path`: The file path where the data will be exported.
    ///
    /// # Errors
    /// - Returns an error if the file could not be created, written to, or flushed.
    pub async fn export_bin(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        self.map_image.read().await.export_bin_file(path).await
    }

    pub(crate) async fn create_snapshot(&self) -> Result<(), Box<dyn std::error::Error>> {
        let image = self.export_full_thumbnail_png().await?;
        let mut file = File::create("snapshot.png").await?;
        file.write_all(&image).await?;
        Ok(())
    }

    pub(crate) async fn diff_snapshot(&self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        if let Ok(mut file) = File::open("snapshot.png").await {
            let mut old_snapshot_encoded = Vec::<u8>::new();
            file.read_to_end(&mut old_snapshot_encoded).await?;
            let old_snapshot = DynamicImage::from_decoder(PngDecoder::new(&mut Cursor::new(
                old_snapshot_encoded,
            ))?)?
                .to_rgba8();
            let map_image = self.map_image_thumbnail.read().await;
            let mut current_snapshot = map_image.buffer.clone();

            for (current_pixel, new_pixel) in
                old_snapshot.pixels().zip(current_snapshot.pixels_mut())
            {
                if *current_pixel == *new_pixel {
                    *new_pixel = Rgba([0, 0, 0, 0]);
                }
            }
            let mut writer = Cursor::new(Vec::<u8>::new());
            current_snapshot.write_with_encoder(PngEncoder::new_with_quality(
                &mut writer,
                CompressionType::Best,
                FilterType::Adaptive,
            ))?;
            let diff_encoded = writer.into_inner();
            File::create("snapshot_diff.png")
                .await
                .unwrap()
                .write_all(&diff_encoded)
                .await
                .unwrap();
            Ok(diff_encoded)
        } else {
            self.export_full_thumbnail_png().await
        }
    }

    pub fn center_pos_to_offset(pos: Vec2D<f32>, angle: CameraAngle) -> Vec2D<u32> {
        Vec2D::new(
            pos.x().round() as i32 - (angle.get_square_side_length() / 2) as i32,
            pos.y().round() as i32 - (angle.get_square_side_length() / 2) as i32,
        )
            .wrap_around_map()
            .cast()
    }
}
