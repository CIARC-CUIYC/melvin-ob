use bitvec::bitbox;
use bitvec::boxed::BitBox;
use bitvec::order::Lsb0;
use image::{ImageBuffer, RgbImage};
use crate::flight_control::camera_state::CameraAngle;
use super::vec2d::Vec2D;

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Bitmap {
    width: u32,
    height: u32,
    pub data: BitBox<usize, Lsb0>,
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
        img.save(output_path).expect("[ERROR] Failed to save the image");
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
                .unwrap_or_else(|| panic!("[FATAL] Index {} - {} not in {len} length", slice_index.0, slice_index.1))
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
        let len = self.data.len();
        for slice_index in self.get_region_slice_indices_from_center(x, y, angle) {
            px += self
                .data
                .get(slice_index.0..slice_index.1)
                .unwrap_or_else(|| panic!("[FATAL] Index {} - {} not in {len} length", slice_index.0, slice_index.1))
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