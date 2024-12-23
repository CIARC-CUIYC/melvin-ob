use crate::flight_control::{
    flight_computer::FlightComputer,
    camera_state::CameraAngle,
    common::{bitmap::Bitmap, img_buffer::Buffer, vec2d::Vec2D},
};
use crate::http_handler::{
    http_client::HTTPClient,
    http_request::request_common::NoBodyHTTPRequestType,
    http_request::shoot_image_get::ShootImageRequest    
};
use bitvec::prelude::BitBox;
use futures::StreamExt;
use image::{imageops::Lanczos3, ImageReader};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    fs::File
};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct CameraController {
    bitmap: Bitmap,
    buffer: Buffer,
}

impl CameraController {
    pub fn new() -> Self {
        let png_bitmap = Bitmap::from_mapsize();
        let png_buffer = Buffer::from_mapsize();
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

    pub async fn shoot_image_to_buffer(
        &mut self,
        httpclient: &HTTPClient,
        controller: &mut FlightComputer<'_>,
        angle: CameraAngle,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let (_, collected_png) = tokio::join!(
            controller.update_observation(),
            self.fetch_image_data(httpclient)
        );
        let decoded_image = self.decode_png_data(
            &collected_png.unwrap_or_else(|e| panic!("[ERROR] PNG coulndt be unwrapped: {e}")),
            angle,
        )?;
        let position = controller.get_current_pos();
        let pos_x = position.x().round() as isize;
        let pos_y = position.y() as isize;
        let angle_const = angle.get_square_radius() as isize;

        // TODO: maybe this can work in parallel?
        for (i, row) in (pos_x - angle_const..pos_x + angle_const).enumerate() {
            for (j, col) in (pos_y - angle_const..pos_y + angle_const).enumerate() {
                let mut coord2d = Vec2D::new(row, col);
                coord2d.wrap_around_map();
                let pixel = decoded_image.get_pixel(i as u32, j as u32);

                self.buffer.save_pixel(coord2d, pixel.0);
            }
        }

        self.bitmap.region_captured(
            Vec2D::new(position.x() as f32, position.y() as f32),
            angle,
            true,
        );

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
}
