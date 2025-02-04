use super::vec2d::{MapSize, Vec2D};
use crate::flight_control::camera_state::CameraAngle;
use bitvec::{bitbox, boxed::BitBox, order::Lsb0};
use fixed::types::{I32F0, I32F32};
use image::{ImageBuffer, RgbImage};
use num::ToPrimitive;
use std::ops::Not;

/// A 2D bitmap structure that uses a bit-packed vector to represent the
/// state of individual pixels. Each pixel is either set (`true`) or unset (`false`).
///
/// This structure provides methods for creating, manipulating, inspecting,
/// and exporting bitmaps.
///
/// # Fields
/// - `width`: A `u32` representing the bitmap's width.
/// - `height`: A `u32` representing the bitmap's height.
/// - `data`: A `BitBox` for storing the state of pixels bit-packed in memory.
#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Bitmap {
    /// The width of the 2D bitmap.
    width: u32,
    /// The height of the 2D bitmap.
    height: u32,
    /// The bit-packed vector that stores pixel states.
    pub data: BitBox<usize, Lsb0>,
}

impl Bitmap {
    /// RGB color representation for red/set bit in exported PNG.
    const RED: [u8; 3] = [255, 0, 0];
    /// RGB color representation for black/unset bit in exported PNG.
    const BLACK: [u8; 3] = [0, 0, 0];

    /// Creates a new `Bitmap` with the specified width and height, where all pixels
    /// are initialized to `false`.
    ///
    /// # Arguments
    /// * `width` - The width of the bitmap.
    /// * `height` - The height of the bitmap.
    ///
    /// # Returns
    /// A new instance of `Bitmap` with the specified dimensions and all pixels unset.
    pub fn new(width: u32, height: u32) -> Self {
        let length = width * height;
        Self {
            width,
            height,
            data: bitbox![usize, Lsb0; 0; length as usize],
        }
    }

    /// Creates a `Bitmap` with dimensions defined by `Vec2D::map_size()`.
    /// All pixels are initialized to `false`.
    ///
    /// # Returns
    /// A `Bitmap` instance corresponding to the constant dimensions from `Vec2D`.
    pub fn from_map_size() -> Self {
        let bitmap_size = Vec2D::<I32F32>::map_size();
        Self::new(
            bitmap_size.x().to_u32().unwrap(),
            bitmap_size.y().to_u32().unwrap(),
        )
    }

    /// Converts 2D `(x, y)` coordinates into an index in the bit-packed array.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel.
    /// * `y` - The y-coordinate of the pixel.
    ///
    /// # Returns
    /// The 1D index corresponding to the specified `x` and `y`.
    ///
    /// # Example
    /// ```rust
    /// let bitmap = Bitmap::new(4, 4);
    /// let index = bitmap.get_bitmap_index(2, 3);
    /// assert_eq!(index, 14);
    /// ```
    fn get_bitmap_index(&self, x: u32, y: u32) -> u32 { y * self.width + x }

    /// Returns the total number of pixels in the bitmap.
    ///
    /// # Returns
    /// The total pixel count as `u32`.
    pub fn len(&self) -> u32 { self.width * self.height }

    /// Checks if the pixel at `(x, y)` is set to `true`.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel.
    /// * `y` - The y-coordinate of the pixel.
    ///
    /// # Returns
    /// `true` if the pixel is set, `false` otherwise.
    pub(crate) fn is_set(&self, x: u32, y: u32) -> bool {
        let index = self.get_bitmap_index(x, y);
        self.data.get(index as usize).is_some_and(|x| *x)
    }

    /// Sets the pixel at `(x, y)` to `true`.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel.
    /// * `y` - The y-coordinate of the pixel.
    ///
    /// # Panics
    /// Panics if `(x, y)` is out of bounds.
    fn set(&mut self, x: u32, y: u32) {
        let index = self.get_bitmap_index(x, y);
        self.data.set(index as usize, true);
    }

    /// Sets the pixel at `(x, y)` to `false`.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel.
    /// * `y` - The y-coordinate of the pixel.
    ///
    /// # Panics
    /// Panics if `(x, y)` is out of bounds.
    fn unset(&mut self, x: u32, y: u32) {
        let index = self.get_bitmap_index(x, y);
        self.data.set(index as usize, false);
    }

    /// Toggles the `true/false` state of the pixel at `(x, y)`.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel.
    /// * `y` - The y-coordinate of the pixel.
    ///
    /// # Panics
    /// Panics if `(x, y)` is out of bounds.
    fn flip(&mut self, x: u32, y: u32) {
        let index = self.get_bitmap_index(x, y);
        let state = self.data.get(index as usize).expect("[FATAL] Index out of bounds!").not();
        self.data.set(index as usize, state);
    }

    /// Sets or unsets a rectangular region based on `pos` and `angle`.
    ///
    /// # Arguments
    /// * `pos` - The center position as `Vec2D<I32F32>`.
    /// * `angle` - Defines the size of the region.
    /// * `set_to` - The value (`true` or `false`) to set for the region.
    ///
    /// # Panics
    /// Panics if index calculations fail.
    pub fn set_region(&mut self, pos: Vec2D<I32F32>, angle: CameraAngle, set_to: bool) {
        let x = I32F0::from_num(pos.x());
        let y = I32F0::from_num(pos.y());
        let slices_vec = self.get_region_slice_indices(x, y, angle);

        for mut slice_index in slices_vec {
            if slice_index.1 >= self.len() {
                slice_index.1 = self.len();
            }
            self.data
                .get_mut(slice_index.0 as usize..slice_index.1 as usize)
                .expect("[FATAL] Index out of bounds!")
                .fill(set_to);
        }
    }

    /// Provides slices representing a region by indices and dimensions.
    ///
    /// # Arguments
    /// * `x` - The center x-coordinate of the region.
    /// * `y` - The center y-coordinate of the region.
    /// * `angle` - Defines the size of the region.
    ///
    /// # Returns
    /// A vector of `(start_index, end_index)` tuples representing slices within the bitmap.
    pub fn get_region_slice_indices(
        &self,
        x: I32F0,
        y: I32F0,
        angle: CameraAngle,
    ) -> Vec<(u32, u32)> {
        let angle_const = i32::from(angle.get_square_side_length() / 2);
        let mut slices = Vec::new();
        let max_height = I32F0::from_num(self.height);
        let max_width = I32F0::from_num(self.width);

        let x_start =
            Vec2D::wrap_coordinate(x - I32F0::from_num(angle_const), max_width).to_u32().unwrap();

        let x_end =
            Vec2D::wrap_coordinate(x + I32F0::from_num(angle_const), max_width).to_u32().unwrap();

        let is_wrapped =
            (i128::from(x_end) - i128::from(x_start)).abs() > i128::from(angle_const * 2);

        let y_i32 = y.to_i32().unwrap();

        for y_it in y_i32 - angle_const..y_i32 + angle_const {
            let wrapped_y =
                Vec2D::wrap_coordinate(I32F0::from_num(y_it), max_height).to_u32().unwrap();

            let start_index = self.get_bitmap_index(x_start, wrapped_y);
            let end_index = self.get_bitmap_index(x_end, wrapped_y);

            if is_wrapped {
                // The row wraps around the width of the map
                let first_part_end_index = self.get_bitmap_index(0, wrapped_y + 1);
                let second_part_start_index = self.get_bitmap_index(0, wrapped_y);
                slices.push((start_index, first_part_end_index));
                slices.push((second_part_start_index, end_index));
            } else {
                // The row is contiguous, no wrapping needed
                slices.push((start_index, end_index));
            }
        }
        slices
    }

    /// Checks whether a region contains at least a specified number of `true` pixels.
    ///
    /// # Arguments
    /// * `pos` - The center position of the region as `Vec2D<I32F32>`.
    /// * `angle` - Defines the region size.
    /// * `min` - The minimum number of `true` pixels required.
    ///
    /// # Returns
    /// `true` if the region contains at least `min` `true` pixels, otherwise `false`.
    pub fn has_sufficient_set_bits(
        &self,
        pos: Vec2D<I32F32>,
        angle: CameraAngle,
        min: usize,
    ) -> bool {
        let mut px = 0;
        let x = I32F0::from_num(pos.x());
        let y = I32F0::from_num(pos.y());
        for slice_index in self.get_region_slice_indices(x, y, angle) {
            px += self
                .data
                .get(slice_index.0 as usize..slice_index.1 as usize)
                .expect("[FATAL] Index out of bounds!")
                .count_ones();
            if px >= min {
                return true;
            }
        }
        false
    }

    /// Exports the bitmap as a PNG image to the specified file path.
    ///
    /// # Arguments
    /// * `output_path` - The path to save the PNG image.
    ///
    /// # Panics
    /// Panics in the event of a file I/O or conversion error.
    pub fn export_to_png(&self, output_path: &str) {
        let mut img: RgbImage = ImageBuffer::new(self.width, self.height);

        // Iterate through the bit vector and set pixel values
        for (index, bit) in self.data.iter().enumerate() {
            let index_u32 = u32::try_from(index).expect("[FATAL] Cast to u32 failed!");
            let x = index_u32 % self.width;
            let y = index_u32 / self.width;
            // Red for true, Black for false
            let pixel = if *bit { Self::RED } else { Self::BLACK };
            img.put_pixel(x, y, image::Rgb(pixel));
        }

        // Save the image to a file
        img.save(output_path).expect("[ERROR] Failed to save the image");
    }
}
