use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::image_data::Buffer;
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::shoot_image_get::ShootImageRequest;
use bit_vec::BitVec;
use futures::StreamExt;
use image::ImageReader;
use tokio::fs::File;
use tokio::io::AsyncWriteExt;
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::Vec2D;

pub struct Bitmap {
    width: usize,
    height: usize,
    pub data: BitVec,
}

pub struct CameraController {}

impl Bitmap {
    pub fn new(width: usize, height: usize) -> Self {
        let size = width * height;
        Bitmap {
            width,
            height,
            data: BitVec::from_elem(size, false),
        }
    }

    // TODO: magic numbers have to be adjusted for the lens used
    pub fn region_captured(&mut self, x: usize, y: usize, angle: CameraAngle) {
        let angle_const = angle.get_square_radius() as usize;
        for row in y-angle_const..y + angle_const {
            for col in x-300..x + angle_const {
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
        flight_computer: &mut FlightComputer<'_>,
        bitmap: &mut Bitmap,
        buffer: &mut Buffer,
    ) -> Result<(), Box<dyn std::error::Error>> {
        flight_computer.update_observation().await;
        let response_stream = ShootImageRequest {}.send_request(httpclient).await?;

        let mut collected_png: Vec<u8> = Vec::new();
        futures::pin_mut!(response_stream);

        while let Some(Ok(chunk_result)) = response_stream.next().await {
            collected_png.extend_from_slice(&chunk_result[..]);
        }

        let decode_image = ImageReader::new(std::io::Cursor::new(collected_png))
            .with_guessed_format()?
            .decode()?
            .to_rgb8();

        let current_pos = flight_computer.get_current_pos();
        let current_x = current_pos.x() as usize;
        let current_y = current_pos.y() as usize;

        let width = decode_image.width();
        let height = decode_image.height();

        for y in 0..width {
            for x in 0..height {
                let pixel = decode_image.get_pixel(x, y);
                // TODO: implement this to map global var somehow
                // copy/past from chatgpt
                let global_x = current_x + (x as usize);
                let global_y = current_y + (y as usize);

                buffer.save_pixel(global_x, global_y, pixel.0)
            }
        }

        // TODO: change this method after rework
        //bitmap.flip_region(current_x, current_y);

        Ok(())
    }
}
