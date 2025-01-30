use image::{GenericImage, GenericImageView};
use std::ops::{Deref, DerefMut};

use crate::flight_control::common::vec2d::Vec2D;

pub(crate) struct SubBuffer<T> {
    pub(crate) buffer: T,
    pub(crate) buffer_size: Vec2D<u32>,
    pub(crate) offset: Vec2D<u32>,
    pub(crate) size: Vec2D<u32>,
}

impl<T> GenericImageView for SubBuffer<T>
where
    T: Deref,
    T::Target: GenericImageView + Sized,
{
    type Pixel = <<T as Deref>::Target as GenericImageView>::Pixel;

    fn dimensions(&self) -> (u32, u32) { (self.size.x(), self.size.y()) }

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
    fn get_pixel_mut(&mut self, x: u32, y: u32) -> &mut Self::Pixel {
        let x_loc = (x + self.offset.x()) % self.buffer_size.x();
        let y_loc = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.get_pixel_mut(x_loc, y_loc)
    }

    fn put_pixel(&mut self, x: u32, y: u32, pixel: Self::Pixel) {
        let x_loc = (x + self.offset.x()) % self.buffer_size.x();
        let y_loc = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.put_pixel(x_loc, y_loc, pixel);
    }

    fn blend_pixel(&mut self, x: u32, y: u32, pixel: Self::Pixel) {
        let x_loc = (x + self.offset.x()) % self.buffer_size.x();
        let y_loc = (y + self.offset.y()) % self.buffer_size.y();
        self.buffer.blend_pixel(x_loc, y_loc, pixel);
    }
}
