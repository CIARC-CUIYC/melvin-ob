use crate::flight_control::common::img_buffer::SubBuffer;
use crate::flight_control::{
    camera_state::CameraAngle,
    common::{bitmap::Bitmap, img_buffer::Buffer, vec2d::Vec2D},
    flight_computer::FlightComputer,
};
use crate::http_handler::{
    http_client::HTTPClient, http_request::request_common::NoBodyHTTPRequestType,
    http_request::shoot_image_get::ShootImageRequest,
};
use futures::StreamExt;
use image::codecs::png::{CompressionType, FilterType, PngDecoder, PngEncoder};
use image::{imageops::Lanczos3, ExtendedColorType, GenericImageView, ImageDecoder, ImageEncoder, ImageReader, Rgba};
use std::io::Cursor;
use std::ops::Deref;
use std::sync::Arc;
use bitvec::boxed::BitBox;
use tokio::sync::RwLock;
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncWriteExt},
};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct MapImage {
    bitmap: Bitmap,
    buffer: Buffer,
}

impl MapImage {
    fn new() -> Self {
        let png_bitmap = Bitmap::from_map_size();
        let png_buffer = Buffer::from_map_size();
        Self {
            bitmap: png_bitmap,
            buffer: png_buffer,
        }
    }

    pub fn view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<MapImage> {
        SubBuffer {
            buffer: &self,
            offset,
            size,
        }
    }
}

impl GenericImageView for MapImage {
    type Pixel = Rgba<u8>;

    fn dimensions(&self) -> (u32, u32) {
        self.buffer.dimensions()
    }

    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel {
        if self.bitmap.is_set(x, y) {
            let pixel = self.buffer.get_pixel(x, y).0;
            Rgba([pixel[0], pixel[1], pixel[2], 0xFF])
        } else {
            Rgba([0, 0, 0, 0])
        }
    }
}


#[derive(Clone)]
pub struct CameraController {
    map_image: Arc<RwLock<MapImage>>,
}

impl CameraController {
    const THUMBNAIL_SCALE_FACTOR: u32 = 25;


    pub fn new() -> Self {
        Self {
            map_image: Arc::new(RwLock::new(MapImage::new()))
        }
    }

    pub async fn from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut file = File::open(path).await?;
        let mut file_buffer = Vec::new();
        file.read_to_end(&mut file_buffer).await?;

        let image = bincode::deserialize(&file_buffer)?;
        Ok(Self {
            map_image: Arc::new(RwLock::new(image))
        })
    }

    pub async fn clone_coverage_bitmap(&self) -> BitBox { self.map_image.read().await.bitmap.data.clone() }

    pub async fn export_bin(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        //let encoded = bincode::serialize(&self)?;
        //let mut bin_file = File::create(path).await?;
        //bin_file.write_all(&encoded).await?;
        //bin_file.flush().await?;
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

    #[allow(clippy::cast_possible_truncation)]
    pub async fn shoot_image_to_buffer(
        &mut self,
        httpclient: &HTTPClient,
        controller: &mut FlightComputer<'_>,
        angle: CameraAngle,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let ((), collected_png) = tokio::join!(
            controller.update_observation(),
            self.fetch_image_data(httpclient)
        );
        let decoded_image = self.decode_png_data(
            &collected_png.unwrap_or_else(|e| panic!("[ERROR] PNG couldn't be unwrapped: {e}")),
            angle,
        )?;
        let position = controller.get_current_pos();
        let pos_x = position.x().round() as i32;
        let pos_y = position.y().round() as i32;
        let angle_const = i32::from(angle.get_square_radius());

        let mut map_image = self.map_image.write().await;
        // TODO: maybe this can work in parallel?
        for (i, row) in (pos_x - angle_const..pos_x + angle_const).enumerate() {
            for (j, col) in (pos_y - angle_const..pos_y + angle_const).enumerate() {
                let row_u32 = u32::try_from(row).expect("[FATAL] Conversion to u32 failed!");
                let col_u32 = u32::try_from(col).expect("[FATAL] Conversion to u32 failed!");
                let coord2d = Vec2D::new(row_u32, col_u32).wrap_around_map();
                let pixel = decoded_image.get_pixel(i as u32, j as u32);
                map_image.buffer.save_pixel(coord2d, pixel.0);
            }
        }
        map_image.bitmap.set_region(Vec2D::new(position.x(), position.y()), angle, true);
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

    pub(crate) async fn export_full_thumbnail_png(&self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let width = Vec2D::<u32>::map_size().x() / Self::THUMBNAIL_SCALE_FACTOR;
        let height = Vec2D::<u32>::map_size().y() / Self::THUMBNAIL_SCALE_FACTOR;
        let map_image = self.map_image.read().await;
        let resized_image = image::imageops::thumbnail(
            map_image.deref(),
            width,
            height,
        );
        let mut writer = Cursor::new(Vec::<u8>::new());
        PngEncoder::new(&mut writer).write_image(&resized_image.into_raw(), width, height, ExtendedColorType::Rgba8)?;

        Ok(writer.into_inner())
    }

    pub(crate) async fn export_thumbnail_png(
        &self,
        position: Vec2D<u32>,
        radius: u32,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let offset = Vec2D::new(position.x() - radius, position.y() - radius).wrap_around_map();

        let map_image = self.map_image.read().await;
        let size = (radius * 2) / Self::THUMBNAIL_SCALE_FACTOR;
        let resized_image = image::imageops::thumbnail(
            &map_image.buffer.view(offset, Vec2D::new(radius * 2, radius * 2)),
            size,
            size,
        );
        let mut writer = Cursor::new(Vec::<u8>::new());
        PngEncoder::new(&mut writer).write_image(&resized_image.into_raw(), size, size, ExtendedColorType::Rgb8)?;

        Ok(writer.into_inner())
    }

    pub(crate) async fn create_snapshot(&self) -> Result<(), Box<dyn std::error::Error>> {
        let image = self.export_full_thumbnail_png().await?;
        let mut file = File::create("snapshot.png").await?;
        file.write_all(&image).await?;
        Ok(())
    }

    pub(crate) async fn diff_snapshot(&self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        if let Ok(mut file) = File::open("snapshot.png").await {
            let mut old_snapshot_encoded = Vec::<u8>::new();
            file.read_to_end(&mut old_snapshot_encoded).await?;
            let width = Vec2D::<u32>::map_size().x() / Self::THUMBNAIL_SCALE_FACTOR;
            let height = Vec2D::<u32>::map_size().y() / Self::THUMBNAIL_SCALE_FACTOR;
            let mut old_snapshot = vec![0u8; (4 * width * height) as usize];
            PngDecoder::new(&mut Cursor::new(old_snapshot_encoded))?.read_image(&mut old_snapshot)?;
            let map_image = self.map_image.read().await;
            let mut current_snapshot: Vec<u8> = image::imageops::thumbnail(
                map_image.deref(),
                width,
                height,
            ).into_raw();


            for i in 0..(old_snapshot.len() / 4) {
                if old_snapshot[i * 4] == current_snapshot[i * 4] &&
                    old_snapshot[i * 4 + 1] == current_snapshot[i * 4 + 1] &&
                    old_snapshot[i * 4 + 2] == current_snapshot[i * 4 + 2] &&
                    old_snapshot[i * 4 + 3] == current_snapshot[i * 4 + 3] {
                    current_snapshot[i * 4] = 0;
                    current_snapshot[i * 4 + 1] = 0;
                    current_snapshot[i * 4 + 1] = 0;
                    current_snapshot[i * 4 + 1] = 0;
                }
            }
            let mut writer = Cursor::new(Vec::<u8>::new());
            PngEncoder::new_with_quality(&mut writer, CompressionType::Best, FilterType::Adaptive).write_image(&current_snapshot, width, height, ExtendedColorType::Rgba8)?;
            let diff_encoded = writer.into_inner();
            File::create("snapshot_diff.png").await.unwrap().write_all(&diff_encoded).await.unwrap();
            Ok(diff_encoded)
        } else {
            self.export_full_thumbnail_png().await
        }
    }
}