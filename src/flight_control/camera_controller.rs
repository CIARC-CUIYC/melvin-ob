use crate::flight_control::{
    camera_state::CameraAngle,
    common::img_buffer::SubBuffer,
    common::{bitmap::Bitmap, vec2d::Vec2D},
    flight_computer::FlightComputer,
};
use crate::http_handler::{
    http_client::HTTPClient, http_request::request_common::NoBodyHTTPRequestType,
    http_request::shoot_image_get::ShootImageRequest,
};
use bitvec::boxed::BitBox;
use futures::StreamExt;
use image::{
    codecs::png::{CompressionType, FilterType, PngDecoder, PngEncoder},
    imageops::Lanczos3,
    DynamicImage, GenericImage, GenericImageView, ImageBuffer, ImageReader, Pixel, Rgb, RgbImage,
    Rgba, RgbaImage,
};
use std::{io::Cursor, sync::Arc, time};
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncWriteExt},
    sync::{Mutex, Notify, RwLock},
};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct MapImageStorage {
    coverage: Bitmap,
    fullsize_buffer: Vec<u8>,
}

pub struct MapImage {
    coverage: Bitmap,
    fullsize_buffer: RgbImage,
    thumbnail_buffer: RgbaImage,
}

impl MapImage {
    pub const THUMBNAIL_SCALE_FACTOR: u32 = 25;

    fn thumbnail_size() -> Vec2D<u32> { Vec2D::map_size() / Self::THUMBNAIL_SCALE_FACTOR }

    fn new() -> Self {
        Self {
            coverage: Bitmap::from_map_size(),
            fullsize_buffer: ImageBuffer::new(Vec2D::map_size().x(), Vec2D::map_size().y()),
            thumbnail_buffer: ImageBuffer::new(
                Self::thumbnail_size().x(),
                Self::thumbnail_size().y(),
            ),
        }
    }

    pub fn fullsize_view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<&MapImage> {
        SubBuffer {
            buffer: self,
            buffer_size: Vec2D::map_size(),
            offset,
            size,
        }
    }

    pub fn fullsize_mut_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Rgb<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &mut self.fullsize_buffer,
            buffer_size: Vec2D::map_size(),
            offset,
            size: Vec2D::map_size(),
        }
    }

    pub fn thumbnail_mut_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Rgba<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &mut self.thumbnail_buffer,
            buffer_size: Self::thumbnail_size(),
            offset,
            size: Self::thumbnail_size(),
        }
    }

    pub fn thumbnail_view(
        &self,
        offset: Vec2D<u32>,
        size: Vec2D<u32>,
    ) -> SubBuffer<&ImageBuffer<Rgba<u8>, Vec<u8>>> {
        SubBuffer {
            buffer: &self.thumbnail_buffer,
            buffer_size: Self::thumbnail_size(),
            offset,
            size,
        }
    }
}

impl GenericImageView for MapImage {
    type Pixel = Rgba<u8>;

    fn dimensions(&self) -> (u32, u32) { self.fullsize_buffer.dimensions() }

    fn get_pixel(&self, x: u32, y: u32) -> Self::Pixel {
        if self.coverage.is_set(x, y) {
            let pixel = self.fullsize_buffer.get_pixel(x, y).0;
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
    pub fn new() -> Self {
        Self {
            map_image: Arc::new(RwLock::new(MapImage::new())),
        }
    }

    pub async fn from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut file = File::open(path).await?;
        let mut file_buffer = Vec::new();
        file.read_to_end(&mut file_buffer).await?;

        let image: MapImageStorage = bincode::deserialize(&file_buffer)?;
        let fullsize_buffer = ImageBuffer::from_raw(
            Vec2D::map_size().x(),
            Vec2D::map_size().y(),
            image.fullsize_buffer,
        )
        .unwrap();

        let mut map_image = MapImage {
            coverage: image.coverage,
            fullsize_buffer,
            thumbnail_buffer: ImageBuffer::new(0, 0),
        };

        let thumbnail_buffer = image::imageops::thumbnail(
            &map_image,
            MapImage::thumbnail_size().x(),
            MapImage::thumbnail_size().y(),
        );
        map_image.thumbnail_buffer = thumbnail_buffer;

        Ok(Self {
            map_image: Arc::new(RwLock::new(map_image)),
        })
    }

    pub async fn clone_coverage_bitmap(&self) -> BitBox {
        self.map_image.read().await.coverage.data.clone()
    }

    #[allow(clippy::cast_sign_loss)]
    fn score_offset(
        decoded_image: &RgbImage,
        base: &MapImage,
        offset_x: u32,
        offset_y: u32,
    ) -> Vec2D<i32> {
        let mut best_score = i32::MIN;
        let mut best_additional_offset = (0, 0).into();
        for additional_offset_x in -2..=2 {
            for additional_offset_y in -2..=2 {
                let pos = Vec2D::new(
                    offset_x as i32 + additional_offset_x,
                    offset_y as i32 + additional_offset_y,
                )
                .wrap_around_map();
                let map_image_view = base.fullsize_view(
                    Vec2D::new(pos.x() as u32, pos.y() as u32),
                    Vec2D::new(decoded_image.width(), decoded_image.height()),
                );
                let score: i32 = map_image_view
                    .pixels()
                    .zip(decoded_image.pixels())
                    .map(|((_, _, existing_pixel), new_pixel)| {
                        if existing_pixel.0[3] == 0 || existing_pixel.to_rgb() == new_pixel.to_rgb()
                        {
                            0
                        } else {
                            -1
                        }
                    })
                    .sum();

                let score = score - additional_offset_x.abs() - additional_offset_y.abs();
                if score > best_score {
                    best_additional_offset = Vec2D::new(additional_offset_x, additional_offset_y);
                    best_score = score;
                }
            }
        }
        best_additional_offset
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub async fn shoot_image_to_buffer(
        &mut self,
        f_cont_locked: Arc<RwLock<FlightComputer>>,
        angle: CameraAngle,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let (position, collected_png) = {
            // TODO: it should be tested if this could be a read lock as well (by not calling update_observation, but current_pos())
            let mut f_cont = f_cont_locked.write().await;
            let client = f_cont.client();
            let ((), collected_png) =
                tokio::join!(f_cont.update_observation(), self.fetch_image_data(&client));
            (f_cont.current_pos(), collected_png)
        };
        let decoded_image = Self::decode_png_data(
            &collected_png.unwrap_or_else(|e| panic!("[ERROR] PNG couldn't be unwrapped: {e}")),
            angle,
        )?;
        let angle_const = angle.get_square_side_length() / 2;
        let pos: Vec2D<i32> = Vec2D::new(
            position.x().round() as i32 - i32::from(angle_const),
            position.y().round() as i32 - i32::from(angle_const),
        )
        .wrap_around_map();

        let mut map_image = self.map_image.write().await;
        let best_offset =
            Self::score_offset(&decoded_image, &map_image, pos.x() as u32, pos.y() as u32);
        let pos = (pos + best_offset).wrap_around_map();

        let mut map_image_view = map_image.fullsize_mut_view(pos.cast());

        map_image_view.copy_from(&decoded_image, 0, 0).unwrap();

        let pos = Vec2D::new(
            pos.x() - MapImage::THUMBNAIL_SCALE_FACTOR as i32 * 2,
            pos.y() - MapImage::THUMBNAIL_SCALE_FACTOR as i32 * 2,
        )
        .wrap_around_map()
        .cast();
        let size = u32::from(angle_const) * 2 + MapImage::THUMBNAIL_SCALE_FACTOR * 4;
        let map_image_view = map_image.fullsize_view(pos, Vec2D::new(size, size));

        let resized_image = image::imageops::thumbnail(
            &map_image_view,
            size / MapImage::THUMBNAIL_SCALE_FACTOR,
            size / MapImage::THUMBNAIL_SCALE_FACTOR,
        );
        map_image
            .thumbnail_mut_view(pos.cast() / MapImage::THUMBNAIL_SCALE_FACTOR)
            .copy_from(&resized_image, 0, 0)
            .unwrap();
        map_image.coverage.set_region(Vec2D::new(position.x(), position.y()), angle, true);
        Ok(())
    }

    async fn fetch_image_data(
        &mut self,
        httpclient: &HTTPClient,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
        let response_stream = ShootImageRequest {}.send_request(httpclient).await?;

        let mut collected_png: Vec<u8> = Vec::new();
        futures::pin_mut!(response_stream);

        while let Some(Ok(chunk_result)) = response_stream.next().await {
            collected_png.extend_from_slice(&chunk_result[..]);
        }

        Ok(collected_png)
    }

    fn decode_png_data(
        collected_png: &[u8],
        angle: CameraAngle,
    ) -> Result<RgbImage, Box<dyn std::error::Error + Send + Sync>> {
        let decoded_image =
            ImageReader::new(Cursor::new(collected_png)).with_guessed_format()?.decode()?.to_rgb8();
        let resized_unit_length = angle.get_square_side_length();

        let resized_image = image::imageops::resize(
            &decoded_image,
            u32::from(resized_unit_length),
            u32::from(resized_unit_length),
            Lanczos3,
        );

        Ok(resized_image)
    }

    pub(crate) async fn export_full_thumbnail_png(
        &self,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let map_image = self.map_image.read().await;
        let mut writer = Cursor::new(Vec::<u8>::new());
        map_image.thumbnail_buffer.write_with_encoder(PngEncoder::new(&mut writer))?;
        Ok(writer.into_inner())
    }

    pub(crate) async fn export_full_view_png(&self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let map_image = self.map_image.read().await;
        let mut writer = Cursor::new(Vec::<u8>::new());
        map_image.fullsize_buffer.write_with_encoder(PngEncoder::new(&mut writer))?;
        Ok(writer.into_inner())
    }

    #[allow(clippy::cast_sign_loss)]
    pub(crate) async fn export_thumbnail_png(
        &self,
        position: Vec2D<i32>,
        radius: u32,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let offset = Vec2D::new(position.x() - radius as i32, position.y() - radius as i32)
            .wrap_around_map();

        let map_image = self.map_image.read().await;
        let thumbnail = map_image.thumbnail_view(
            Vec2D::new(
                offset.x() as u32 / MapImage::THUMBNAIL_SCALE_FACTOR,
                offset.y() as u32 / MapImage::THUMBNAIL_SCALE_FACTOR,
            ),
            Vec2D::new(
                (radius * 2) / MapImage::THUMBNAIL_SCALE_FACTOR,
                (radius * 2) / MapImage::THUMBNAIL_SCALE_FACTOR,
            ),
        );

        let mut thumbnail_image = RgbaImage::new(thumbnail.width(), thumbnail.width());
        thumbnail_image.copy_from(&thumbnail, 0, 0).unwrap();
        let mut writer = Cursor::new(Vec::<u8>::new());
        thumbnail_image.write_with_encoder(PngEncoder::new(&mut writer))?;

        Ok(writer.into_inner())
    }

    pub(crate) async fn create_snapshot_thumb(&self) -> Result<(), Box<dyn std::error::Error>> {
        let image = self.export_full_thumbnail_png().await?;
        let mut file = File::create("snapshot.png").await?;
        file.write_all(&image).await?;
        Ok(())
    }

    pub(crate) async fn create_snapshot_full(&self) -> Result<(), Box<dyn std::error::Error>> {
        println!("[INFO] Exporting Full-View PNG...");
        let start_time = chrono::Utc::now();
        let image = self.export_full_view_png().await?;
        let mut file = File::create("snapshot.png").await?;
        file.write_all(&image).await?;
        println!(
            "[INFO] Exported Full-View PNG in {} s!",
            (chrono::Utc::now() - start_time).num_seconds()
        );
        Ok(())
    }

    pub(crate) async fn diff_snapshot(&self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        if let Ok(mut file) = File::open("snapshot.png").await {
            let mut old_snapshot_encoded = Vec::<u8>::new();
            file.read_to_end(&mut old_snapshot_encoded).await?;
            let old_snapshot = DynamicImage::from_decoder(PngDecoder::new(&mut Cursor::new(
                old_snapshot_encoded,
            ))?)?
            .to_rgba8();
            let map_image = self.map_image.read().await;
            let mut current_snapshot = map_image.thumbnail_buffer.clone();

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
            File::create("snapshot_diff.png")
                .await
                .unwrap()
                .write_all(&diff_encoded)
                .await
                .unwrap();
            Ok(diff_encoded)
        } else {
            self.export_full_thumbnail_png().await
        }
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub async fn execute_acquisition_cycle(
        this: Arc<Mutex<Self>>,
        f_cont_locked: Arc<RwLock<FlightComputer>>,
        end_time_locked: Arc<Mutex<chrono::DateTime<chrono::Utc>>>,
        last_img_kill: Arc<Notify>,
        image_max_dt: f32,
        lens: CameraAngle,
    ) {
        let mut last_image_flag = false;
        let mut pic_count = 0;

        loop {
            {
                let mut c_cont = this.lock().await;
                match c_cont.shoot_image_to_buffer(Arc::clone(&f_cont_locked), lens).await {
                    Ok(()) => {
                        pic_count += 1;
                        println!(
                            "[INFO] Took {pic_count}. picture in cycle at {}",
                            chrono::Utc::now()
                        );
                    }
                    Err(e) => println!("[ERROR] Couldn't take picture: {e}"),
                };
            }
            if last_image_flag {
                return;
            }
            let sleep_time = {
                let end_time = end_time_locked.lock().await;
                if chrono::Utc::now() + chrono::TimeDelta::seconds(image_max_dt as i64) > *end_time
                {
                    last_image_flag = true;
                    time::Duration::from_secs((*end_time - chrono::Utc::now()).num_seconds() as u64)
                } else {
                    time::Duration::from_secs_f32(image_max_dt.floor())
                }
            };
            tokio::select! {
                () = tokio::time::sleep(sleep_time) => {},
                () = last_img_kill.notified() => {
                    last_image_flag = true;
                }
            }
        }
    }
}
