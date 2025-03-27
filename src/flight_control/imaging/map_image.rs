use super::{file_based_buffer::FileBackedBuffer, sub_buffer::SubBuffer};
use crate::flight_control::common::vec2d::{MapSize, Vec2D};
use image::{
    DynamicImage, EncodableLayout, GenericImage, GenericImageView, ImageBuffer, Pixel,
    PixelWithColorType, Rgb, RgbImage,
    codecs::png::{CompressionType, FilterType, PngDecoder, PngEncoder},
    imageops,
};
use std::{
    io::{BufReader, Cursor},
    ops::{Deref, DerefMut},
    path::Path,
};
use tokio::{fs::File, io::AsyncReadExt};

/// Represents an extracted and encoded image with metadata.
///
/// This struct contains information about the region of the image
/// that was extracted, its dimensions, and the encoded image data.
///
/// # Fields
/// * `offset` - The top-left corner of the extracted image region in the original image.
/// * `size` - The dimensions (width and height) of the extracted region.
/// * `data` - The encoded image data as a vector of bytes.
pub(crate) struct EncodedImageExtract {
    /// The top-left corner of the extracted image region in the original image.
    pub(crate) offset: Vec2D<u32>,
    /// The dimensions (width and height) of the extracted region.
    pub(crate) size: Vec2D<u32>,
    /// The encoded image data as a vector of bytes.
    pub(crate) data: Vec<u8>,
}

/// Trait representing operations for working with map images.
///
/// This generic trait allows manipulating and extracting data from images
/// stored as 2D pixel buffers. It provides methods to work with sub-regions of
/// the image, export images in PNG format, update specific areas, and more.
///
/// # Type Parameters
/// * `Pixel` - The pixel type used by the image, which implements `PixelWithColorType`.
/// * `Container` - The container storing pixel subcomponents, implementing `Deref` and `DerefMut`.
/// * `ViewSubBuffer` - A view into a sub-region of the image, implementing `GenericImageView`.
pub(crate) trait MapImage {
    /// The type of the pixels in the image.
    type Pixel: PixelWithColorType;

    /// The container for the pixel data.
    type Container: Deref<Target = [<Self::Pixel as Pixel>::Subpixel]> + DerefMut;

    /// A view of a sub-region of the image.
    type ViewSubBuffer: GenericImageView<Pixel: PixelWithColorType>;

    /// Provides a mutable view of the image at the specified offset.
    ///
    /// # Arguments
    /// * `offset` - The top-left corner of the requested region.
    ///
    /// # Returns
    /// A `SubBuffer` representing the specified region of the image.
    fn mut_vec_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Self::Pixel, Self::Container>>;

    /// Provides a view of a sub-region of the image.
    ///
    /// # Arguments
    /// * `offset` - The top-left corner of the requested region.
    /// * `size` - The dimensions of the requested region.
    ///
    /// # Returns
    /// A `SubBuffer` representing the specified region of the image.
    fn vec_view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<&Self::ViewSubBuffer>;

    /// Returns a reference to the entire image buffer.
    ///
    /// # Returns
    /// A reference to the image buffer.
    fn buffer(&self) -> &ImageBuffer<Self::Pixel, Self::Container>;

    /// Exports the entire image buffer as a PNG.
    ///
    /// This method encodes the image as a PNG and returns the encoded byte array along
    /// with metadata about the image. The encoded data is stored in an `EncodedImageExtract`
    /// struct that contains the image's offset, size, and encoded data.
    ///
    /// # Returns
    /// An `EncodedImageExtract` containing the offset, size, and encoded image data.
    ///
    /// # Errors
    /// Returns an error if the PNG encoding process fails.
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

    /// Exports a specific sub-region of the image as a PNG.
    ///
    /// This method extracts the specified sub-region of the image, encodes it as a PNG,
    /// and returns it as an `EncodedImageExtract`. The sub-region to be extracted is defined
    /// by the provided `offset` and `size`.
    ///
    /// # Arguments
    /// * `offset` - The top-left corner of the region to export.
    /// * `size` - The dimensions of the region to export, specified as a width and height.
    ///
    /// # Returns
    /// An `EncodedImageExtract` containing the offset, size, and encoded image data of the sub-region.
    ///
    /// # Errors
    /// Returns an error if the PNG encoding process fails.
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
        >::new(size.x(), size.y());
        area_image.copy_from(&area_view, 0, 0).unwrap();
        let mut writer = Cursor::new(Vec::<u8>::new());
        area_image.write_with_encoder(PngEncoder::new(&mut writer))?;
        Ok(EncodedImageExtract { offset, size, data: writer.into_inner() })
    }

    /// Saves the current image buffer as a snapshot in PNG format.
    ///
    /// This method writes the image's content to the file at the specified path in PNG format.
    ///
    /// # Arguments
    /// * `path` - The file path where the snapshot should be saved.
    ///
    /// # Returns
    /// Returns `Ok(())` if the save operation is successful.
    /// Returns an error if the save process fails.
    fn create_snapshot<P: AsRef<Path>>(&self, path: P) -> Result<(), Box<dyn std::error::Error>>
    where [<Self::Pixel as Pixel>::Subpixel]: EncodableLayout {
        self.buffer().save(path)?;
        Ok(())
    }

    /// Updates a specific sub-region of the image with the given data.
    ///
    /// This method copies the content of `image` into the corresponding sub-region of the current
    /// image buffer, starting from the specified `offset`.
    ///
    /// # Arguments
    /// * `offset` - The top-left corner of the target sub-region to update.
    /// * `image` - The new image data to copy into the target sub-region.
    fn update_area<I: GenericImageView<Pixel = Self::Pixel>>(
        &mut self,
        offset: Vec2D<u32>,
        image: &I,
    ) {
        self.mut_vec_view(offset).copy_from(image, 0, 0).unwrap();
    }
}

/// A struct representing a full-sized map image.
///
/// This struct manages the full-sized map image which includes
/// a coverage bitmap and an image buffer backed by a memory-mapped file.
/// It provides functionality to open and handle the image buffer efficiently.
///
/// # Fields
/// * `coverage` - A `Bitmap` instance representing the coverage of the map image.
/// * `image_buffer` - An `ImageBuffer` containing the RGB pixel data, backed by a `FileBackedBuffer`.
pub(crate) struct FullsizeMapImage {
    /// The image buffer containing the pixel data, backed by a file.
    image_buffer: ImageBuffer<Rgb<u8>, FileBackedBuffer>,
}

pub(crate) struct OffsetZonedObjectiveImage {
    offset: Vec2D<u32>,
    image_buffer: ImageBuffer<Rgb<u8>, Vec<u8>>,
}

impl OffsetZonedObjectiveImage {
    pub fn new(offset: Vec2D<u32>, dimensions: Vec2D<u32>) -> Self {
        Self { offset, image_buffer: ImageBuffer::new(dimensions.x(), dimensions.y()) }
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_wrap)]
    pub fn update_area<I: GenericImageView<Pixel = Rgb<u8>>>(
        &mut self,
        offset: Vec2D<u32>,
        image: &I,
    ) {
        for x in 0..image.width() {
            let offset_x = (offset.x() + x) as i32;
            let relative_offset_x =
                Vec2D::wrap_coordinate(offset_x - self.offset.x() as i32, Vec2D::map_size().x())
                    as u32;

            if relative_offset_x >= self.image_buffer.width() {
                continue;
            }
            for y in 0..image.height() {
                let offset_y = (offset.y() + y) as i32;
                let relative_offset_y = Vec2D::wrap_coordinate(
                    offset_y - self.offset.y() as i32,
                    Vec2D::map_size().y(),
                ) as u32;

                if relative_offset_y >= self.image_buffer.height() {
                    continue;
                }
                *self.image_buffer.get_pixel_mut(relative_offset_x, relative_offset_y) =
                    image.get_pixel(x, y);
            }
        }
    }

    fn export_as_png(&self) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        let mut writer = Cursor::new(Vec::<u8>::new());
        self.image_buffer.write_with_encoder(PngEncoder::new(&mut writer))?;
        Ok(EncodedImageExtract {
            offset: self.offset,
            size: Vec2D::new(self.image_buffer.width(), self.image_buffer.height()),
            data: writer.into_inner(),
        })
    }
}

impl GenericImageView for OffsetZonedObjectiveImage {
    type Pixel = Rgb<u8>;

    fn dimensions(&self) -> (u32, u32) { self.image_buffer.dimensions() }

    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel { *self.image_buffer.get_pixel(x, y) }
}

impl MapImage for OffsetZonedObjectiveImage {
    type Pixel = Rgb<u8>;
    type Container = Vec<u8>;
    type ViewSubBuffer = OffsetZonedObjectiveImage;

    fn mut_vec_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Self::Pixel, Self::Container>> {
        SubBuffer {
            buffer: &mut self.image_buffer,
            buffer_size: u32::map_size(),
            offset,
            size: u32::map_size(),
        }
    }

    fn vec_view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<&Self::ViewSubBuffer> {
        SubBuffer { buffer: self, buffer_size: u32::map_size(), offset, size }
    }

    fn buffer(&self) -> &ImageBuffer<Self::Pixel, Self::Container> { &self.image_buffer }
}

impl FullsizeMapImage {
    /// Opens a full-sized map image from a file.
    ///
    /// This function initializes a `FileBackedBuffer` for efficient memory-mapped file access
    /// and creates an `ImageBuffer` using the data in the mapped file.
    ///
    /// # Arguments
    /// * `path` - The file path of the image to open.
    ///
    /// # Returns
    /// An instance of `FullsizeMapImage` with the coverage bitmap initialized
    /// and the image buffer mapped to the file.
    ///
    /// # Panics
    /// This function will panic if:
    /// * The `FileBackedBuffer` cannot be created.
    /// * The `ImageBuffer` cannot be created from the `FileBackedBuffer`.
    pub(crate) fn open<P: AsRef<Path>>(path: P) -> Self {
        let fullsize_buffer_size: usize =
            (u32::map_size().x() as usize) * (u32::map_size().y() as usize) * 3;
        let file_based_buffer = FileBackedBuffer::open(path, fullsize_buffer_size).unwrap();
        Self {
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
    /// The pixel type used by the image buffer, in this case, `Rgba<u8>`.
    type Pixel = Rgb<u8>;

    /// Returns the dimensions of the image buffer as a tuple `(width, height)`.
    ///
    /// # Returns
    /// A tuple containing the width and height of the image buffer.
    fn dimensions(&self) -> (u32, u32) { self.image_buffer.dimensions() }

    /// Retrieves the pixel at the given `(x, y)` coordinates.
    ///
    /// If the pixel is covered (as checked by the coverage bitmap), the corresponding
    /// pixel data from the image buffer will be returned with an alpha value of `0xFF`
    /// (fully opaque). Otherwise, a transparent black pixel `[0, 0, 0, 0]` is returned.
    ///
    /// # Arguments
    /// * `x` - The horizontal coordinate of the pixel.
    /// * `y` - The vertical coordinate of the pixel.
    ///
    /// # Returns
    /// An `Rgba<u8>` pixel that is either from the image buffer (if covered) or
    /// a transparent black pixel (if not covered).
    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel { *self.image_buffer.get_pixel(x, y) }
}

impl MapImage for FullsizeMapImage {
    /// The pixel type for the image, in this case `Rgb<u8>`.
    type Pixel = Rgb<u8>;
    /// The container type for the pixel data, in this case `FileBackedBuffer` used for memory-mapped file access.
    type Container = FileBackedBuffer;
    /// The view type for a sub-region of the image, implemented as `FullsizeMapImage`.
    type ViewSubBuffer = FullsizeMapImage;

    /// Provides a mutable view of the image buffer at the specified offset.
    ///
    /// # Arguments
    /// * `offset` - The top-left corner of the region to view.
    ///
    /// # Returns
    /// A `SubBuffer` containing a mutable reference to the image buffer
    /// starting from the specified offset.
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

    /// Provides a view of a sub-region of the image buffer.
    ///
    /// # Arguments
    /// * `offset` - The top-left corner of the region to view.
    /// * `size` - The dimensions of the region to view.
    ///
    /// # Returns
    /// A `SubBuffer` containing a reference to the `FullsizeMapImage` starting
    /// from the specified offset and region size.
    fn vec_view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<&FullsizeMapImage> {
        SubBuffer { buffer: self, buffer_size: u32::map_size(), offset, size }
    }

    /// Returns a reference to the entire image buffer.
    ///
    /// # Returns
    /// A reference to the `ImageBuffer` containing the RGB pixel data.
    fn buffer(&self) -> &ImageBuffer<Self::Pixel, Self::Container> { &self.image_buffer }
}

/// Represents a thumbnail image generated from a full-size map image.
///
/// This struct is designed to manage scaled-down versions of map images,
/// which are useful for generating previews or comparing snapshots.
pub(crate) struct ThumbnailMapImage {
    /// The underlying image buffer storing the pixel data of the thumbnail.
    image_buffer: RgbImage,
}

impl MapImage for ThumbnailMapImage {
    /// The pixel type used, which is RGBA with 8-bit sub-pixels.
    type Pixel = Rgb<u8>;
    /// The container type for the pixel data, represented as a vector of bytes.
    type Container = Vec<u8>;
    /// The view type for sub-regions of the thumbnail, implemented as an `ImageBuffer`.
    type ViewSubBuffer = ImageBuffer<Rgb<u8>, Vec<u8>>;

    /// Provides a mutable view of the thumbnail at the specified offset.
    ///
    /// # Arguments
    /// * `offset` - The top-left corner of the requested sub-region.
    ///
    /// # Returns
    /// A `SubBuffer` representing the specified sub-region of the thumbnail.
    fn mut_vec_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Rgb<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &mut self.image_buffer,
            buffer_size: Self::thumbnail_size(),
            offset,
            size: Self::thumbnail_size(),
        }
    }

    /// Provides a view of a sub-region of the thumbnail.
    ///
    /// # Arguments
    /// * `offset` - The top-left corner of the requested sub-region.
    /// * `size` - The dimensions of the requested sub-region.
    ///
    /// # Returns
    /// A `SubBuffer` representing the specified sub-region of the thumbnail.
    fn vec_view(
        &self,
        offset: Vec2D<u32>,
        size: Vec2D<u32>,
    ) -> SubBuffer<&ImageBuffer<Rgb<u8>, Vec<u8>>> {
        SubBuffer { buffer: &self.image_buffer, buffer_size: Self::thumbnail_size(), offset, size }
    }

    /// Returns a reference to the entire image buffer of the thumbnail.
    ///
    /// # Returns
    /// A reference to the image buffer storing the thumbnail's pixel data.
    fn buffer(&self) -> &ImageBuffer<Self::Pixel, Self::Container> { &self.image_buffer }
}

impl ThumbnailMapImage {
    /// Defines the scale factor for generating a thumbnail from a full-size map image.
    ///
    /// The dimensions of the thumbnail are calculated by dividing the full-sized map
    /// dimensions by this constant.
    pub(crate) const THUMBNAIL_SCALE_FACTOR: u32 = 25;

    /// Calculates the size of the thumbnail based on the full-size map dimensions.
    ///
    /// This method uses the `THUMBNAIL_SCALE_FACTOR` to scale down the map size.
    ///
    /// # Returns
    /// A `Vec2D<u32>` representing the dimensions of the thumbnail.
    pub(crate) fn thumbnail_size() -> Vec2D<u32> { u32::map_size() / Self::THUMBNAIL_SCALE_FACTOR }

    /// Generates a thumbnail from a given full-sized map image.
    ///
    /// This method scales down the provided `FullsizeMapImage` to create a thumbnail
    /// using the pre-defined `thumbnail_size`.
    ///
    /// # Arguments
    /// * `fullsize_map_image` - A reference to the `FullsizeMapImage` to be converted.
    ///
    /// # Returns
    /// A `ThumbnailMapImage` containing the scaled-down image.
    pub(crate) fn from_fullsize(fullsize_map_image: &FullsizeMapImage) -> Self {
        Self {
            image_buffer: imageops::thumbnail(
                fullsize_map_image,
                Self::thumbnail_size().x(),
                Self::thumbnail_size().y(),
            ),
        }
    }

    /// Generates a thumbnail from a previously saved snapshot.
    ///
    /// If the snapshot file exists, it is loaded and converted into a thumbnail.
    /// If it does not exist, a blank image with the dimensions of the thumbnail is created.
    ///
    /// # Arguments
    /// * `snapshot_path` - The file path to the snapshot PNG.
    ///
    /// # Returns
    /// A `ThumbnailMapImage` containing either the loaded thumbnail image or a blank thumbnail.
    pub(crate) fn from_snapshot<P: AsRef<Path>>(snapshot_path: P) -> Self {
        let image_buffer = if let Ok(file) = std::fs::File::open(snapshot_path) {
            DynamicImage::from_decoder(PngDecoder::new(&mut BufReader::new(file)).unwrap())
                .unwrap()
                .to_rgb8()
        } else {
            ImageBuffer::new(Self::thumbnail_size().x(), Self::thumbnail_size().y())
        };
        Self { image_buffer }
    }

    /// Computes the difference between the current thumbnail and a snapshot.
    ///
    /// This method compares the pixel data of the current thumbnail against a previously
    /// saved snapshot and creates a new image showing the differences. Pixels that
    /// are identical are marked as transparent, and differing pixels retain their values.
    ///
    /// If the snapshot file does not exist, the current thumbnail is exported as a PNG.
    ///
    /// # Arguments
    /// * `base_snapshot_path` - The file path to the base snapshot PNG.
    ///
    /// # Returns
    /// An `EncodedImageExtract` containing the diff image as a PNG.
    ///
    /// # Errors
    /// Returns an error if the snapshot file cannot be read or the PNG encoding fails.
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
            .to_rgb8();
            let mut current_snapshot = self.image_buffer.clone();

            for (current_pixel, new_pixel) in
                old_snapshot.pixels().zip(current_snapshot.pixels_mut())
            {
                if *current_pixel == *new_pixel {
                    *new_pixel = Rgb([0, 0, 0]);
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

#[cfg(test)]
mod tests {
    use crate::flight_control::camera_state::CameraAngle;

    use super::*;

    #[test]
    fn test_overflow() {
        let mut fullsize_image = FullsizeMapImage::open("tmp.bin");

        let angle = CameraAngle::Normal;
        let area_size = u32::from(angle.get_square_side_length());
        let offset = Vec2D::new(
            Vec2D::<u32>::map_size().x() - area_size / 2,
            Vec2D::<u32>::map_size().y() - area_size / 2,
        );

        let mut area_image: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::new(area_size, area_size);
        for x in 0..area_size {
            for y in 0..area_size {
                *area_image.get_pixel_mut(x, y) = Rgb([
                    (x % 0xFF) as u8,
                    (y % 0xFF) as u8,
                    ((x + 7 + y * 3) % 130) as u8,
                ]);
            }
        }
        fullsize_image.update_area(offset, &area_image);
        let assert_area_edge = |fs_offset: Vec2D<u32>, area_offset: Vec2D<u32>, size: u32| {
            let fs_view = fullsize_image.vec_view(fs_offset, Vec2D::new(size, size));
            let mut fs_image: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::new(size, size);
            fs_image.copy_from(&fs_view, 0, 0).unwrap();
            let area_view = area_image.view(area_offset.x(), area_offset.y(), size, size);
            assert_eq!(fs_image.as_raw(), area_view.to_image().as_raw());
        };
        assert_area_edge(
            Vec2D::new(0, 0),
            Vec2D::new(area_size / 2, area_size / 2),
            area_size / 2,
        );
        assert_area_edge(
            Vec2D::new(
                Vec2D::<u32>::map_size().x() - area_size / 2,
                Vec2D::<u32>::map_size().y() - area_size / 2,
            ),
            Vec2D::new(0, 0),
            area_size / 2,
        );
        assert_area_edge(offset, Vec2D::new(0, 0), area_size);
    }
}
