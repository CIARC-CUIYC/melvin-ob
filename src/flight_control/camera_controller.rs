use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::{camera_state::CameraAngle, common::Vec2D, image_data::Buffer};
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::shoot_image_get::ShootImageRequest;
use bitvec::{bitbox, prelude::{BitBox, Lsb0}};
use futures::StreamExt;
use image::{imageops::Lanczos3, ImageBuffer, ImageReader, RgbImage};
use tokio::fs::File;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Bitmap {
    width: u32,
    height: u32,
    pub data: BitBox<usize, Lsb0>,
}

#[derive(serde::Serialize, serde::Deserialize)]
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

    pub fn export_to_png(&self, output_path: &str) {
        let mut img: RgbImage = ImageBuffer::new(self.width, self.height);

        // Iterate through the bit vector and set pixel values
        for (index, bit) in self.data.iter().enumerate() {
            let x = (index % self.width as usize) as u32;
            let y = (index / self.width as usize) as u32;
            let pixel = if *bit { [255, 0, 0] } else { [0, 0, 0] }; // Red for true, Black for false
            img.put_pixel(x, y, image::Rgb(pixel));
        }

        // Save the image to a file
        img.save(output_path).expect("Failed to save the image");
    }

    pub fn size(&self) -> usize { (self.width * self.height) as usize }

    pub fn region_captured(&mut self, pos: Vec2D<f32>, angle: CameraAngle, set_to: bool) {
        let x = pos.x() as isize;
        let y = pos.y() as isize;
        let slices_vec = self.get_region_slice_indices_from_center(x, y, angle);
        let len = self.data.len();
        for mut slice_index in slices_vec {
            if slice_index.1 >= self.data.len() { slice_index.1 = self.data.len() }
            self.data
                .get_mut(slice_index.0..slice_index.1)
                .unwrap_or_else(|| panic!("Unwrapped index range {} - {} is out of bounds for array of {len} length", slice_index.0, slice_index.1))
                .fill(set_to);
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
        
        let x_start = Vec2D::wrap_coordinate(x - angle_const, max_width);
        let x_end = Vec2D::wrap_coordinate(x + angle_const, max_width);
        let is_wrapped = (x_end - x_start).abs() > (angle_const * 2);
        
        for y_it in y - angle_const..y + angle_const {
            let wrapped_y = Vec2D::wrap_coordinate(y_it, max_height);
            
            let start_index = self.get_bitmap_index(x_start as usize, wrapped_y as usize);
            let end_index = self.get_bitmap_index(x_end as usize, wrapped_y as usize);

            if is_wrapped {
                // The row wraps around the width of the map
                let wrapped_y_plus_one = Vec2D::wrap_coordinate(wrapped_y + 1, max_height);
                let first_part_end_index = self.get_bitmap_index(0, (wrapped_y + 1) as usize);
                let second_part_start_index = self.get_bitmap_index(0, wrapped_y as usize);
                slices.push((start_index, first_part_end_index));
                slices.push((second_part_start_index, end_index));
            } else {
                // The row is contiguous, no wrapping needed
                slices.push((start_index, end_index));
            }
        }
        slices
    }

    pub fn enough_ones_in_square(&self, pos: Vec2D<f64>, angle: CameraAngle, min: usize) -> bool {
        let mut px = 0;
        let x = pos.x() as isize;
        let y = pos.y() as isize;
        for slice_index in self.get_region_slice_indices_from_center(x, y, angle) {
            px += self
                .data
                .get(slice_index.0..slice_index.1)
                .unwrap()
                .count_ones();
            if px >= min {
                return true;
            }
        }
        false
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
        let decoded_image = self.decode_png_data(&collected_png.unwrap(), angle)?;
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
