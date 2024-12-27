use image::GenericImageView;
use super::vec2d::Vec2D;

// TODO: this could be useful as soon as metadata for pixels is necessary
/*
pub struct PixelData {
    rgb: [u8; 3],
    // NOTE: possible Metadata field could be useful here
}
 */

/// A 2D raster buffer to store pixel data. Each pixel is represented by an RGB triplet.
///
/// This structure provides methods for creating a buffer, saving pixels, and computing
/// the 1D index of pixels based on their 2D coordinates.
#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Buffer {
    /// The width of the buffer (in pixels).
    width: u32,
    /// The height of the buffer (in pixels).
    height: u32,
    /// The 2D buffer data storing RGB values as `[u8; 3]` for each pixel.
    data: Vec<[u8; 3]>,
}

impl Buffer {
    /// Creates a new `Buffer` with the specified `width` and `height`.
    /// All pixels are initialized to black (`[0, 0, 0]`).
    ///
    /// # Arguments
    /// * `width` - The width of the buffer.
    /// * `height` - The height of the buffer.
    ///
    /// # Returns
    /// A new instance of `Buffer` initialized with black pixels.
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            data: vec![[0, 0, 0]; (width * height) as usize],
            width,
            height,
        }
    }

    /// Creates a `Buffer` based on the dimensions of the map size defined in `Vec2D`.
    /// All pixels are initialized to black (`[0, 0, 0]`).
    ///
    /// # Returns
    /// A new `Buffer` with dimensions provided by `Vec2D::map_size()`.
    pub fn from_map_size() -> Self {
        let map_size = Vec2D::<u32>::map_size();
        Self::new(map_size.x(), map_size.y())
    }

    /// Saves an RGB pixel at the specified wrapped position `(x, y)`.
    ///
    /// # Arguments
    /// * `wrapped_pos` - The 2D wrapped position of the pixel as a `Vec2D<u32>`.
    /// * `rgb` - The RGB color to be saved, specified as an array `[u8; 3]`.
    pub fn save_pixel(&mut self, wrapped_pos: Vec2D<u32>, rgb: [u8; 3]) {
        let index = self.get_buffer_index(wrapped_pos.x(), wrapped_pos.y());
        self.data[index as usize] = rgb;
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
    /// let buffer = Buffer::new(4, 4);
    /// let index = buffer.get_buffer_index(2, 3);
    /// assert_eq!(index, 14);
    /// ```
    pub fn get_buffer_index(&self, x: u32, y: u32) -> u32 {
        y * self.width + x
    }
}

impl GenericImageView for Buffer {
    type Pixel = image::Rgb<u8>;

    fn dimensions(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel {
        image::Rgb(self.data[(y * self.width + x) as usize])
    }
}
