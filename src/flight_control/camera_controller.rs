use std::io::Cursor;
use crate::flight_control::{
    camera_state::CameraAngle,
    common::{bitmap::Bitmap, img_buffer::Buffer, vec2d::Vec2D},
    flight_computer::FlightComputer,
};
use crate::http_handler::{
    http_client::HTTPClient, http_request::request_common::NoBodyHTTPRequestType,
    http_request::shoot_image_get::ShootImageRequest,
};
use bitvec::prelude::BitBox;
use futures::StreamExt;
use image::{imageops::Lanczos3, GenericImageView, ImageReader};
use image::codecs::jpeg::JpegEncoder;
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncWriteExt},
};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct CameraController {
    bitmap: Bitmap,
    buffer: Buffer,
}

impl CameraController {
    pub fn new() -> Self {
        let png_bitmap = Bitmap::from_map_size();
        let png_buffer = Buffer::from_map_size();
        Self {
            bitmap: png_bitmap,
            buffer: png_buffer,
        }
    }

    pub async fn from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut file = File::open(path).await?;
        let mut file_buffer = Vec::new();
        file.read_to_end(&mut file_buffer).await?;

        let camera = bincode::deserialize(&file_buffer)?;
        Ok(camera)
    }

    pub fn map_data_ref(&self) -> &BitBox { &self.bitmap.data }
    pub fn bitmap_ref(&self) -> &Bitmap { &self.bitmap }
    pub fn buffer_ref(&self) -> &Buffer { &self.buffer }

    pub async fn export_bin(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let encoded = bincode::serialize(&self)?;
        let mut bin_file = File::create(path).await?;
        bin_file.write_all(&encoded).await?;
        bin_file.flush().await?;
        Ok(())
    }

    pub async fn shoot_image_to_disk(
        &mut self,
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

    #[allow(clippy::cast_possible_truncation)]
    pub async fn shoot_image_to_buffer(
        &mut self,
        httpclient: &HTTPClient,
        controller: &mut FlightComputer<'_>,
        angle: CameraAngle,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let ((), collected_png) = tokio::join!(
            controller.update_observation(),
            self.fetch_image_data(httpclient)
        );
        let decoded_image = self.decode_png_data(
            &collected_png.unwrap_or_else(|e| panic!("[ERROR] PNG couldn't be unwrapped: {e}")),
            angle,
        )?;
        let position = controller.get_current_pos();
        let pos_x = position.x().round() as i32;
        let pos_y = position.y().round() as i32;
        let angle_const = i32::from(angle.get_square_radius());

        // TODO: maybe this can work in parallel?
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
        self.bitmap.set_region(Vec2D::new(position.x(), position.y()), angle, true);
        Ok(())
    }

    async fn fetch_image_data(
        &mut self,
        httpclient: &HTTPClient,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let response_stream = ShootImageRequest {}.send_request(httpclient).await?;

        let mut collected_png: Vec<u8> = Vec::new();
        futures::pin_mut!(response_stream);

        while let Some(Ok(chunk_result)) = response_stream.next().await {
            collected_png.extend_from_slice(&chunk_result[..]);
        }

        Ok(collected_png)
    }

    fn decode_png_data(
        &mut self,
        collected_png: &[u8],
        angle: CameraAngle,
    ) -> Result<image::RgbImage, Box<dyn std::error::Error>> {
        let decoded_image = ImageReader::new(std::io::Cursor::new(collected_png))
            .with_guessed_format()?
            .decode()?
            .to_rgb8();
        let resized_unit_length = angle.get_square_unit_length();

        let resized_image = image::imageops::resize(
            &decoded_image,
            u32::from(resized_unit_length),
            u32::from(resized_unit_length),
            Lanczos3,
        );

        Ok(resized_image)
    }

    pub(crate) fn export_jpg(
        &mut self,
        offset: (u32, u32),
        size: (u32, u32),
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        const SCALE_FACTOR: u32 = 25;
        let resized_image = image::imageops::resize(
            self.buffer.view(offset.0, offset.1, size.0, size.1).inner(),
            u32::from(size.0 / SCALE_FACTOR),
            u32::from(size.1 / SCALE_FACTOR),
            Lanczos3,
        );
        let mut writer = Cursor::new(Vec::<u8>::new());
        JpegEncoder::new(&mut writer).encode_image(&resized_image)?;

        Ok(writer.into_inner())
    }
}