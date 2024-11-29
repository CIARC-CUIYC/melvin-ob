use crate::flight_control::{camera_state::CameraAngle, common::Vec2D, image_data::Buffer};
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::shoot_image_get::ShootImageRequest;
use bitvec::prelude::Lsb0;
use bitvec::{bitbox, prelude::BitBox};
use futures::StreamExt;
use image::{imageops::Lanczos3, ImageReader};
use tokio::fs::File;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Bitmap {
    width: u32,
    height: u32,
    pub data: BitBox,
}

pub struct CameraController {
    bitmap: Bitmap,
    buffer: Buffer,
}

impl Bitmap {
    pub fn new(width: u32, height: u32) -> Self {
        let len = width * height;
        Self {
            width,
            height,
            data: bitbox![usize, Lsb0; 0; len as usize],
        }
    }

    pub fn from_mapsize() -> Self {
        let bitmap_size = Vec2D::<u32>::map_size();
        Self::new(bitmap_size.x(), bitmap_size.y())
    }

    pub fn size(&self) -> usize { (self.width * self.height) as usize }

    pub fn region_captured(&mut self, pos: Vec2D<f32>, angle: CameraAngle) {
        let x = pos.x() as isize;
        let y = pos.y() as isize;
        let slices_vec = self.get_region_slice_indices_from_center(x, y, angle);
        for slice_index in slices_vec {
            self.data
                .get_mut(slice_index.0..slice_index.1)
                .unwrap()
                .fill(true);
        }
    }

    pub fn get_region_slice_indices_from_center(
        &self,
        x: isize,
        y: isize,
        angle: CameraAngle,
    ) -> Vec<(usize, usize)> {
        let angle_const = angle.get_square_radius() as isize;
        let mut slices = Vec::new();
        let max_height = self.height as isize;
        let max_width = self.width as isize;

        for y_it in y - angle_const..y + angle_const {
            let wrapped_y = Vec2D::wrap_coordinate(y_it, max_height);
            let x_start = Vec2D::wrap_coordinate(x - angle_const, max_width);
            let x_end = Vec2D::wrap_coordinate(x + angle_const, max_width);

            let start_index = self.get_bitmap_index(x_start as usize, wrapped_y as usize);
            let end_index = self.get_bitmap_index(x_end as usize, wrapped_y as usize);

            if (x_end - x_start).abs() <= (angle_const * 2) {
                // The row is contiguous, no wrapping needed
                slices.push((start_index, end_index));
            } else {
                // The row wraps around the width of the map
                let first_part_end_index = self.get_bitmap_index(0, (wrapped_y + 1) as usize);
                let second_part_start_index = self.get_bitmap_index(0, wrapped_y as usize);
                slices.push((start_index, first_part_end_index));
                slices.push((second_part_start_index, end_index));
            }
        }
        slices
    }

    pub fn is_square_empty(&self, pos: Vec2D<f64>, angle: CameraAngle) -> bool {
        let angle_const = angle.get_square_radius() as isize;
        let x = pos.x() as isize;
        let y = pos.y() as isize;
        for slice_index in self.get_region_slice_indices_from_center(x, y, angle) {
            if self.data.get(slice_index.0..slice_index.1).unwrap().any() {
                return false;
            }
        }
        true
    }

    fn is_photographed(&self, x: usize, y: usize) -> Result<bool, String> {
        if x >= self.width as usize || y >= self.height as usize {
            // TODO: approaching edge
            return Err("EDGE".to_string());
        }
        Ok(self.check_pixel(x, y))
    }

    fn check_pixel(&self, x: usize, y: usize) -> bool {
        let index = self.get_bitmap_index(x, y);
        self.data.get(index).is_some_and(|x| x == true)
    }

    fn set_pixel(&mut self, x: usize, y: usize, state: bool) {
        let index = self.get_bitmap_index(x, y);
        self.data.set(index, state);
    }

    // Converts 2D (x, y) coordinates to a 1D index of memory
    fn get_bitmap_index(&self, x: usize, y: usize) -> usize { y * self.width as usize + x }
}

impl CameraController {
    pub fn new() -> Self {
        let png_bitmap = Bitmap::from_mapsize();
        let png_buffer = Buffer::new();
        Self {
            bitmap: png_bitmap,
            buffer: png_buffer,
        }
    }

    pub async fn from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut file = File::open(path).await?;
        let mut buffer = Vec::new();
        file.read_to_end(&mut buffer).await?;

        let (bitmap, buffer_data): (Bitmap, Buffer) = bincode::deserialize(&buffer)?;

        Ok(CameraController {
            bitmap,
            buffer: buffer_data,
        })
    }

    pub fn map_ref(&self) -> &BitBox { &self.bitmap.data }

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
        position: Vec2D<isize>,
        angle: CameraAngle,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let collected_png = self.fetch_image_data(httpclient).await?;
        let decoded_image = self.decode_png_data(&collected_png, angle)?;

        let angle_const = angle.get_square_radius() as isize;
        for (i, row) in (position.x() - angle_const..position.x() + angle_const).enumerate() {
            for (j, col) in (position.x() - angle_const..position.x() + angle_const).enumerate() {
                let mut coord2d = Vec2D::new(row as f32, col as f32);
                coord2d.wrap_around_map();
                let pixel = decoded_image.get_pixel(i as u32, j as u32);

                self.buffer.save_pixel(coord2d, pixel.0);
            }
        }
        self.bitmap
            .region_captured(Vec2D::new(position.x() as f32, position.y() as f32), angle);

        Ok(())
    }

    pub async fn export_bin(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let data_to_serialize = (&self.bitmap, &self.buffer);
        let encoded = bincode::serialize(&data_to_serialize)?;
        let mut bin_file = File::create(path).await?;
        bin_file.write_all(&encoded).await?;
        bin_file.flush().await?;
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
