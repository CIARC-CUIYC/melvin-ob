use image::{GenericImage, GenericImageView};
use std::ops::{Deref, DerefMut};

use crate::flight_control::common::vec2d::Vec2D;

/// A sub-region of an image, represented as a buffer with bounds.
/// This struct allows for accessing and modifying a subsection of the image efficiently.
pub(crate) struct SubBuffer<T> {
    /// The underlying image buffer.
    pub(crate) buffer: T,
    /// The size of the entire buffer in pixels.
    pub(crate) buffer_size: Vec2D<u32>,
    /// The offset of this sub-buffer within the overall buffer.
    pub(crate) offset: Vec2D<u32>,
    /// The dimensions of the sub-buffer.
    pub(crate) size: Vec2D<u32>,
}

impl<T> GenericImageView for SubBuffer<T>
where
    T: Deref,
    T::Target: GenericImageView + Sized,
{
    type Pixel = <<T as Deref>::Target as GenericImageView>::Pixel;

    /// Returns the dimensions of the sub-buffer in pixels.
    fn dimensions(&self) -> (u32, u32) { (self.size.x(), self.size.y()) }

    /// Fetches a pixel from the sub-buffer at the specified coordinates.
    ///
    /// # Arguments
    ///
    /// * `x` - The x-coordinate of the pixel to fetch.
    /// * `y` - The y-coordinate of the pixel to fetch.
    ///
    /// # Returns
    ///
    /// The pixel at the specified coordinates.
    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel {
        let x_loc = (x + self.offset.x()) % self.buffer_size.x();
        let y_loc = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.get_pixel(x_loc, y_loc)
    }
}

impl<T> GenericImage for SubBuffer<T>
where
    T: DerefMut,
    T::Target: GenericImage + Sized,
{
    /// Fetches a mutable reference to a pixel within the sub-buffer at the specified coordinates.
    ///
    /// # Arguments
    ///
    /// * `x` - The x-coordinate of the pixel to fetch.
    /// * `y` - The y-coordinate of the pixel to fetch.
    ///
    /// # Returns
    ///
    /// A mutable reference to the pixel at the specified coordinates.
    #[allow(deprecated)]
    fn get_pixel_mut(&mut self, x: u32, y: u32) -> &mut Self::Pixel {
        let x_loc = (x + self.offset.x()) % self.buffer_size.x();
        let y_loc = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.get_pixel_mut(x_loc, y_loc)
    }

    /// Writes a pixel to the sub-buffer at the specified coordinates.
    ///
    /// # Arguments
    ///
    /// * `x` - The x-coordinate where the pixel will be written.
    /// * `y` - The y-coordinate where the pixel will be written.
    /// * `pixel` - The pixel to write at the specified coordinates.
    fn put_pixel(&mut self, x: u32, y: u32, pixel: Self::Pixel) {
        let x_loc = (x + self.offset.x()) % self.buffer_size.x();
        let y_loc = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.put_pixel(x_loc, y_loc, pixel);
    }

    /// Blends a pixel with the pixel at the specified coordinates within the sub-buffer.
    ///
    /// # Arguments
    ///
    /// * `x` - The x-coordinate where the pixel will be blended.
    /// * `y` - The y-coordinate where the pixel will be blended.
    /// * `pixel` - The pixel to blend at the specified coordinates.
    #[allow(deprecated)]
    fn blend_pixel(&mut self, x: u32, y: u32, pixel: Self::Pixel) {
        let x_loc = (x + self.offset.x()) % self.buffer_size.x();
        let y_loc = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.blend_pixel(x_loc, y_loc, pixel);
    }
}
