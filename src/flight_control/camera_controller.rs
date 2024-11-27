use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::Vec2D;
use crate::flight_control::image_data::Buffer;
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::shoot_image_get::ShootImageRequest;
use bit_vec::BitVec;
use futures::StreamExt;
use image::imageops::Lanczos3;
use image::ImageReader;
use tokio::fs::File;
use tokio::io::AsyncWriteExt;

pub struct Bitmap {
    width: usize,
    height: usize,
    pub data: BitVec,
}

pub struct CameraController {
    bitmap: Bitmap,
    buffer: Buffer,
}

impl Bitmap {
    pub fn new() -> Self {
        let bitmap_size = Vec2D::<usize>::map_size();
        let bitmap_area = bitmap_size.x * bitmap_size.y;
        Self {
            width: bitmap_size.x,
            height: bitmap_size.y,
            data: BitVec::from_elem(bitmap_area, false),
        }
    }

    pub fn size(&self) -> usize {
        self.width * self.height
    }

    pub fn region_captured(&mut self, x: isize, y: isize, angle: CameraAngle) {
        let angle_const = angle.get_square_radius() as isize;
        for row in y - angle_const..y + angle_const {
            for col in x - angle_const..x + angle_const {
                let mut coord2d = Vec2D::new(row as f64, col as f64);
                coord2d.wrap_around_map();
                self.set_pixel(coord2d.x() as usize, coord2d.y() as usize, true);
            }
        }
    }

    fn is_photographed(&self, x: usize, y: usize) -> Result<bool, String> {
        if x >= self.width || y >= self.height {
            // TODO: approaching edge
            return Err("EDGE".to_string());
        }
        Ok(self.check_pixel(x, y))
    }

    fn check_pixel(&self, x: usize, y: usize) -> bool {
        let index = self.get_bitmap_index(x, y);
        self.data.get(index).unwrap_or(false)
    }

    fn set_pixel(&mut self, x: usize, y: usize, state: bool) {
        let index = self.get_bitmap_index(x, y);
        self.data.set(index, state);
    }

    // Converts 2D (x, y) coordinates to a 1D index of memory
    fn get_bitmap_index(&self, x: usize, y: usize) -> usize {
        y * self.width + x
    }
}

impl CameraController {
    pub fn new() -> Self {
        let png_bitmap = Bitmap::new();
        let png_buffer = Buffer::new();
        Self {
            bitmap: png_bitmap,
            buffer: png_buffer,
        }
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
        position: Vec2D<usize>,
        angle: CameraAngle,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let collected_png = self.fetch_image_data(httpclient).await?;
        let decoded_image = self.decode_png_data(&collected_png, angle)?;

        let angle_const = angle.get_square_radius() as usize;
        for row in position.y.saturating_div(angle_const)..position.y.saturating_add(angle_const) {
            for col in
                position.x.saturating_div(angle_const)..position.x.saturating_add(angle_const)
            {
                let pixel = decoded_image.get_pixel(u32::try_from(col)?, u32::try_from(row)?);

                let global_x = position.x + (col);
                let global_y = position.y + (row);

                let global_position = Vec2D::new_usize(global_x, global_y);

                self.buffer.save_pixel(global_position, pixel.0);
                // TODO: implement region capture correctly
                // self.bitmap.region_captured()
            }
        }

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
        const RESIZED_HEIGHT: u32 = 1000;
        const RESIZED_WIDTH: u32 = 1000;

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
