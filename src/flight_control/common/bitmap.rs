use super::vec2d::Vec2D;
use crate::flight_control::camera_state::CameraAngle;
use bitvec::{bitbox, boxed::BitBox, order::Lsb0};
use image::{ImageBuffer, RgbImage};
use std::ops::Not;

/// A 2D bitmap structure that uses a bit-packed vector to represent the
/// state of individual pixels. Each pixel is either set (`true`) or unset (`false`).
///
/// # Fields
/// - `width`: A `u32` representing the 2D bitmap width.
/// - `height`: A `u32` representing the 2D bitmap height.
/// - `data`: A `BitBox` storing the actual bitmap data.
/// 
/// This structure provides methods for manipulating pixel data, checking
/// regions, and exporting the bitmap to a PNG image.
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
    const RED: [u8; 3] = [255, 0, 0];
    const BLACK: [u8; 3] = [0, 0, 0];

    /// Creates a new `Bitmap` with the specified `width` and `height`.
    /// All pixels in the bitmap are initialized to `false`.
    ///
    /// # Arguments
    /// * `width` - The width of the bitmap.
    /// * `height` - The height of the bitmap.
    ///
    /// # Returns
    /// A new instance of `Bitmap`.
    pub fn new(width: u32, height: u32) -> Self {
        let length = width * height;
        Self {
            width,
            height,
            data: bitbox![usize, Lsb0; 0; length as usize],
        }
    }

    /// Creates a `Bitmap` based on the constant dimensions of the map size defined in `Vec2D`.
    /// All pixels are initialized to `false`.
    ///
    /// # Returns
    /// A new `Bitmap` with dimensions provided by `Vec2D::map_size()`.
    pub fn from_map_size() -> Self {
        let bitmap_size = Vec2D::<u32>::map_size();
        Self::new(bitmap_size.x(), bitmap_size.y())
    }

    /// Converts the 2D `(x, y)` coordinate to a 1D index in the bit-packed array.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate.
    /// * `y` - The y-coordinate.
    ///
    /// # Returns
    /// The 1D index as `u32`.
    /// 
    /// # Example
    /// ```rust
    /// let bitmap = Bitmap::new(4, 4);
    /// let index = bitmap.get_bitmap_index(2, 3);
    /// assert_eq!(index, 14);
    /// ```
    fn get_bitmap_index(&self, x: u32, y: u32) -> u32 { y * self.width + x }

    /// Returns the total number of pixels in the bitmap.
    /// This is equivalent to the product of its `width` and `height`.
    ///
    /// # Returns
    /// The total number of pixels as `u32`.
    pub fn len(&self) -> u32 { self.width * self.height }

    /// Checks whether the pixel at the specified `(x, y)` coordinates is set.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel.
    /// * `y` - The y-coordinate of the pixel.
    ///
    /// # Returns
    /// `true` if the pixel is set, `false` if the pixel is out of bounds or unset.
    fn is_set(&self, x: u32, y: u32) -> bool {
        let index = self.get_bitmap_index(x, y);
        self.data.get(index as usize).is_some_and(|x| *x)
    }

    /// Sets the pixel at the given `(x, y)` coordinates to `true`.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel to set.
    /// * `y` - The y-coordinate of the pixel to set.
    ///
    /// # Panics
    /// This panics if `(x, y)` is out of bounds.
    fn set(&mut self, x: u32, y: u32) {
        let index = self.get_bitmap_index(x, y);
        self.data.set(index as usize, true);
    }

    /// Sets the pixel at the given `(x, y)` coordinates to `false`.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel to unset.
    /// * `y` - The y-coordinate of the pixel to unset.
    ///
    /// # Panics
    /// This panics if `(x, y)` is out of bounds.
    fn unset(&mut self, x: u32, y: u32) {
        let index = self.get_bitmap_index(x, y);
        self.data.set(index as usize, false);
    }

    /// Toggles the pixel state at the given `(x, y)` coordinates.
    /// This flips the state of the pixel: `true` becomes `false`,
    /// and `false` becomes `true`.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the pixel to flip.
    /// * `y` - The y-coordinate of the pixel to flip.
    ///
    /// # Panics
    /// This panics if `(x, y)` is out of bounds.
    fn flip(&mut self, x: u32, y: u32) {
        let index = self.get_bitmap_index(x, y);
        let state = self
            .data
            .get(index as usize)
            .expect("[FATAL] Index out of bounds!")
            .not();
        self.data.set(index as usize, state);
    }

    /// Updates a region of the bitmap to a given `set_to` state.
    /// The region is determined by its center and the Camera Angle of capture.
    ///
    /// # Arguments
    /// * `pos` - The center position as `Vec2D<f32>`.
    /// * `angle` - A `CameraAngle` defining the region's size.
    /// * `set_to` - The state to set the region's pixels to (`true` or `false`).
    ///
    /// # Panics
    /// This can panic due to conversion errors or bugs left in `get_region_slice_indices`.
    #[allow(clippy::cast_possible_truncation)]
    pub fn set_region(&mut self, pos: Vec2D<f32>, angle: CameraAngle, set_to: bool) {
        let x = pos.x().round() as i32;
        let y = pos.y().round() as i32;
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

    /// Retrieves the start and end indices of all slices within a region,
    /// based on the center position `(x, y)` and the Camera Angle `angle`.
    ///
    /// # Arguments
    /// * `x` - The x-coordinate of the region's center.
    /// * `y` - The y-coordinate of the region's center.
    /// * `angle` - A `CameraAngle` defining the region's size.
    ///
    /// # Returns
    /// A `Vec` of tuples, where each tuple specifies a `(start_index, end_index)`.
    ///
    /// # Panics
    /// This can panic due to conversion errors or bugs left.
    pub fn get_region_slice_indices(&self, x: i32, y: i32, angle: CameraAngle) -> Vec<(u32, u32)> {
        let angle_const = i32::from(angle.get_square_radius());
        let mut slices = Vec::new();
        let max_height = i32::try_from(self.height).expect("[FATAL] Cast to i32 failed!");
        let max_width = i32::try_from(self.width).expect("[FATAL] Cast to i32 failed!");

        let x_start = u32::try_from(Vec2D::wrap_coordinate(x - angle_const, max_width))
            .expect("[FATAL] Cast to u32 failed!");
        let x_end = u32::try_from(Vec2D::wrap_coordinate(x + angle_const, max_width))
            .expect("[FATAL] Cast to u32 failed!");

        let is_wrapped =
            (i128::from(x_end) - i128::from(x_start)).abs() > i128::from(angle_const * 2);

        for y_it in y - angle_const..y + angle_const {
            let wrapped_y = u32::try_from(Vec2D::wrap_coordinate(y_it, max_height))
                .expect("[FATAL] Cast to u32 failed!");

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

    /// Checks if a region of the bitmap has at least `min` bits set to `true`.
    ///
    /// # Arguments
    /// * `pos` - The center position of the region as `Vec2D<f32>`.
    /// * `angle` - A `CameraAngle` defining the region's size.
    /// * `min` - The minimum number of bits that need to be set.
    ///
    /// # Returns
    /// `true` if the region contains at least `min` bits set, otherwise `false`.
    ///
    /// # Panics
    /// This can panic due to conversion errors or bugs left in `get_region_slice_indices`.
    #[allow(clippy::cast_possible_truncation)]
    pub fn has_sufficient_set_bits(&self, pos: Vec2D<f32>, angle: CameraAngle, min: usize) -> bool {
        let mut px = 0;
        let x = pos.x().round() as i32;
        let y = pos.y().round() as i32;
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

    /// Exports the bitmap to a PNG image file.
    /// Each pixel is represented as either red (`true`) or black (`false`).
    ///
    /// # Arguments
    /// * `output_path` - The path where the PNG image will be saved.
    ///
    /// # Panics
    /// This can panic due to conversion errors or file I/O errors.
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
