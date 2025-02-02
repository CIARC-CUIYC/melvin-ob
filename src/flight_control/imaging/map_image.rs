use std::{
    io::{BufReader, Cursor},
    ops::{Deref, DerefMut},
    path::Path,
};

use image::{
    codecs::png::{CompressionType, FilterType, PngDecoder, PngEncoder},
    DynamicImage, EncodableLayout, GenericImage, GenericImageView, ImageBuffer, Pixel,
    PixelWithColorType, Rgb, Rgba, RgbaImage,
};
use tokio::{fs::File, io::AsyncReadExt};

use crate::flight_control::common::{bitmap::Bitmap, vec2d::Vec2D};
use crate::flight_control::common::vec2d::MapSize;
use super::{file_based_buffer::FileBackedBuffer, sub_buffer::SubBuffer};

pub(crate) struct EncodedImageExtract {
    pub(crate) offset: Vec2D<u32>,
    pub(crate) size: Vec2D<u32>,
    pub(crate) data: Vec<u8>,
}

pub(crate) trait MapImage {
    type Pixel: PixelWithColorType;
    type Container: Deref<Target = [<Self::Pixel as Pixel>::Subpixel]> + DerefMut;
    type ViewSubBuffer: GenericImageView<Pixel: PixelWithColorType>;

    fn mut_vec_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Self::Pixel, Self::Container>>;

    fn vec_view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<&Self::ViewSubBuffer>;

    fn buffer(&self) -> &ImageBuffer<Self::Pixel, Self::Container>;

    fn export_as_png(&self) -> Result<EncodedImageExtract, Box<dyn std::error::Error>>
    where [<Self::Pixel as Pixel>::Subpixel]: EncodableLayout {
        let mut writer = Cursor::new(Vec::<u8>::new());
        let buffer = self.buffer();
        buffer.write_with_encoder(PngEncoder::new(&mut writer))?;
        Ok(EncodedImageExtract {
            offset: Vec2D::new(0, 0),
            size: Vec2D::new(buffer.width(), buffer.height()),
            data: writer.into_inner(),
        })
    }

    #[allow(clippy::cast_sign_loss)]
    fn export_area_as_png(
        &self,
        offset: Vec2D<u32>,
        size: Vec2D<u32>,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>>
    where
        [<<Self::ViewSubBuffer as GenericImageView>::Pixel as Pixel>::Subpixel]: EncodableLayout,
    {
        let area_view = self.vec_view(offset, size);

        let mut area_image = ImageBuffer::<
            <Self::ViewSubBuffer as GenericImageView>::Pixel,
            Vec<<<Self::ViewSubBuffer as GenericImageView>::Pixel as Pixel>::Subpixel>,
        >::new(area_view.width(), area_view.width());
        area_image.copy_from(&area_view, 0, 0).unwrap();
        let mut writer = Cursor::new(Vec::<u8>::new());
        area_image.write_with_encoder(PngEncoder::new(&mut writer))?;
        Ok(EncodedImageExtract {
            offset,
            size,
            data: writer.into_inner(),
        })
    }

    fn create_snapshot<P: AsRef<Path>>(&self, path: P) -> Result<(), Box<dyn std::error::Error>>
    where [<Self::Pixel as Pixel>::Subpixel]: EncodableLayout {
        self.buffer().save(path)?;
        Ok(())
    }

    fn update_area<I: GenericImageView<Pixel = Self::Pixel>>(
        &mut self,
        offset: Vec2D<u32>,
        image: I,
    ) {
        self.mut_vec_view(offset).copy_from(&image, 0, 0).unwrap();
    }
}

pub(crate) struct FullsizeMapImage {
    pub(crate) coverage: Bitmap,
    image_buffer: ImageBuffer<Rgb<u8>, FileBackedBuffer>,
}

impl FullsizeMapImage {
    pub(crate) fn open<P: AsRef<Path>>(path: P) -> Self {
        let fullsize_buffer_size: usize = (u32::map_size().x() as usize)
            * (u32::map_size().y() as usize)
            * 3;
        let file_based_buffer = FileBackedBuffer::open(path, fullsize_buffer_size).unwrap();
        Self {
            coverage: Bitmap::from_map_size(),
            image_buffer: ImageBuffer::from_raw(
                u32::map_size().x(),
                u32::map_size().y(),
                file_based_buffer,
            )
            .unwrap(),
        }
    }
}

impl GenericImageView for FullsizeMapImage {
    type Pixel = Rgba<u8>;

    fn dimensions(&self) -> (u32, u32) { self.image_buffer.dimensions() }

    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel {
        if self.coverage.is_set(x, y) {
            let pixel = self.image_buffer.get_pixel(x, y).0;
            Rgba([pixel[0], pixel[1], pixel[2], 0xFF])
        } else {
            Rgba([0, 0, 0, 0])
        }
    }
}

impl MapImage for FullsizeMapImage {
    type Pixel = Rgb<u8>;
    type Container = FileBackedBuffer;
    type ViewSubBuffer = FullsizeMapImage;

    fn vec_view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<&FullsizeMapImage> {
        SubBuffer {
            buffer: self,
            buffer_size: u32::map_size(),
            offset,
            size,
        }
    }

    fn mut_vec_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Rgb<u8>, FileBackedBuffer>> {
        SubBuffer {
            buffer: &mut self.image_buffer,
            buffer_size: u32::map_size(),
            offset,
            size: u32::map_size(),
        }
    }

    fn buffer(&self) -> &ImageBuffer<Self::Pixel, Self::Container> { &self.image_buffer }
}

pub(crate) struct ThumbnailMapImage {
    image_buffer: RgbaImage,
}

impl MapImage for ThumbnailMapImage {
    type Pixel = Rgba<u8>;
    type ViewSubBuffer = ImageBuffer<Rgba<u8>, Vec<u8>>;
    type Container = Vec<u8>;

    fn mut_vec_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Rgba<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &mut self.image_buffer,
            buffer_size: Self::thumbnail_size(),
            offset,
            size: Self::thumbnail_size(),
        }
    }

    fn vec_view(
        &self,
        offset: Vec2D<u32>,
        size: Vec2D<u32>,
    ) -> SubBuffer<&ImageBuffer<Rgba<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &self.image_buffer,
            buffer_size: Self::thumbnail_size(),
            offset,
            size,
        }
    }

    fn buffer(&self) -> &ImageBuffer<Self::Pixel, Self::Container> { &self.image_buffer }
}

impl ThumbnailMapImage {
    pub(crate) const THUMBNAIL_SCALE_FACTOR: u32 = 25;

    pub(crate) fn thumbnail_size() -> Vec2D<u32> {
        u32::map_size() / Self::THUMBNAIL_SCALE_FACTOR
    }

    pub(crate) fn from_fullsize(fullsize_map_image: &FullsizeMapImage) -> Self {
        Self {
            image_buffer: image::imageops::thumbnail(
                fullsize_map_image,
                Self::thumbnail_size().x(),
                Self::thumbnail_size().y(),
            ),
        }
    }

    pub(crate) fn from_snapshot<P: AsRef<Path>>(snapshot_path: P) -> Self {
        let image_buffer = if let Ok(file) = std::fs::File::open(snapshot_path) {
            DynamicImage::from_decoder(PngDecoder::new(&mut BufReader::new(file)).unwrap())
                .unwrap()
                .to_rgba8()
        } else {
            ImageBuffer::new(Self::thumbnail_size().x(), Self::thumbnail_size().y())
        };
        Self { image_buffer }
    }

    pub(crate) async fn diff_with_snapshot<P: AsRef<Path>>(
        &self,
        base_snapshot_path: P,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        if let Ok(mut file) = File::open(base_snapshot_path).await {
            let mut old_snapshot_encoded = Vec::<u8>::new();
            file.read_to_end(&mut old_snapshot_encoded).await?;
            let old_snapshot = DynamicImage::from_decoder(PngDecoder::new(&mut Cursor::new(
                old_snapshot_encoded,
            ))?)?
            .to_rgba8();
            let mut current_snapshot = self.image_buffer.clone();

            for (current_pixel, new_pixel) in
                old_snapshot.pixels().zip(current_snapshot.pixels_mut())
            {
                if *current_pixel == *new_pixel {
                    *new_pixel = Rgba([0, 0, 0, 0]);
                }
            }
            let mut writer = Cursor::new(Vec::<u8>::new());
            current_snapshot.write_with_encoder(PngEncoder::new_with_quality(
                &mut writer,
                CompressionType::Best,
                FilterType::Adaptive,
            ))?;
            let diff_encoded = writer.into_inner();
            Ok(EncodedImageExtract {
                offset: Vec2D::new(0, 0),
                size: u32::map_size() / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
                data: diff_encoded,
            })
        } else {
            self.export_as_png()
        }
    }
}
