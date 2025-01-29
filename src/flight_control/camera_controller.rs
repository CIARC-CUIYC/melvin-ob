use crate::console_communication::console_messenger::ConsoleMessenger;
use crate::flight_control::{
    camera_state::CameraAngle,
    common::img_buffer::SubBuffer,
    common::{bitmap::Bitmap, vec2d::Vec2D},
    flight_computer::FlightComputer,
};
use crate::http_handler::http_request::daily_map_post::DailyMapRequest;
use crate::http_handler::http_request::objective_image_post::ObjectiveImageRequest;
use crate::http_handler::http_request::request_common::MultipartBodyHTTPRequestType;
use crate::http_handler::{
    http_client::HTTPClient, http_request::request_common::NoBodyHTTPRequestType,
    http_request::shoot_image_get::ShootImageRequest,
};
use bitvec::boxed::BitBox;
use core::slice;
use chrono::TimeDelta;
use futures::StreamExt;
use image::{
    codecs::png::{CompressionType, FilterType, PngDecoder, PngEncoder},
    imageops::Lanczos3,
    DynamicImage, GenericImage, GenericImageView, ImageBuffer, ImageReader, Pixel, Rgb, RgbImage,
    Rgba, RgbaImage,
};
use num::traits::Float;
use std::ffi::c_void;
use std::ops::{Deref, DerefMut};
use std::os::fd::AsRawFd;
use std::path::Path;
use std::ptr::null_mut;
use std::{io::Cursor, sync::Arc};
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncWriteExt},
    sync::{Mutex, Notify, RwLock},
};
use crate::DT_0_STD;

pub struct EncodedImageExtract {
    pub(crate) offset: Vec2D<u32>,
    pub(crate) size: Vec2D<u32>,
    pub(crate) data: Vec<u8>,
}

pub(crate) struct FileBasedBuffer {
    file: std::fs::File,
    ptr: *mut u8,
    length: usize,
}

impl FileBasedBuffer {
    fn open<T: AsRef<Path>>(path: T, length: usize) -> Self {
        let file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .read(true)
            .truncate(false)
            .open(path)
            .unwrap();
        let res = unsafe { libc::ftruncate(file.as_raw_fd(), length as i64) };
        if res != 0 {
            panic!("ftruncate failed");
        }
        let ptr = unsafe {
            libc::mmap(
                null_mut(),
                length,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED | libc::MAP_FILE,
                file.as_raw_fd(),
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            panic!("mmap failed");
        }
        FileBasedBuffer {
            file,
            length,
            ptr: ptr as *mut u8,
        }
    }
}

impl Drop for FileBasedBuffer {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.ptr as *mut c_void, self.length);
        }
    }
}

unsafe impl Send for FileBasedBuffer {}
unsafe impl Sync for FileBasedBuffer {}

impl Deref for FileBasedBuffer {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        return unsafe { slice::from_raw_parts(self.ptr, self.length) };
    }
}

impl DerefMut for FileBasedBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        return unsafe { slice::from_raw_parts_mut(self.ptr, self.length) };
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct MapImageStorage {
    coverage: Bitmap,
}

pub struct FullsizeMapImage {
    coverage: Bitmap,
    image_buffer: ImageBuffer<Rgb<u8>, FileBasedBuffer>,
}

impl FullsizeMapImage {
    fn open<P: AsRef<Path>>(path: P) -> Self {
        let fullsize_buffer_size: usize =
            Vec2D::<usize>::map_size().x() * Vec2D::<usize>::map_size().y() * 3;
        let file_based_buffer = FileBasedBuffer::open(path, fullsize_buffer_size);
        Self {
            coverage: Bitmap::from_map_size(),
            image_buffer: ImageBuffer::from_raw(
                Vec2D::map_size().x(),
                Vec2D::map_size().y(),
                file_based_buffer,
            )
            .unwrap(),
        }
    }

    pub fn vec_view(&self, offset: Vec2D<u32>, size: Vec2D<u32>) -> SubBuffer<&FullsizeMapImage> {
        SubBuffer {
            buffer: self,
            buffer_size: Vec2D::map_size(),
            offset,
            size,
        }
    }

    pub fn mut_vec_view(
        &mut self,
        offset: Vec2D<u32>,
    ) -> SubBuffer<&mut ImageBuffer<Rgb<u8>, FileBasedBuffer>> {
        SubBuffer {
            buffer: &mut self.image_buffer,
            buffer_size: Vec2D::map_size(),
            offset,
            size: Vec2D::map_size(),
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

pub struct ThumbnailMapImage {
    image_buffer: RgbaImage,
}

impl ThumbnailMapImage {
    pub const THUMBNAIL_SCALE_FACTOR: u32 = 25;

    fn thumbnail_size() -> Vec2D<u32> { Vec2D::map_size() / Self::THUMBNAIL_SCALE_FACTOR }

    pub fn from_fullsize(fullsize_map_image: &FullsizeMapImage) -> Self {
        Self {
            image_buffer: image::imageops::thumbnail(
                fullsize_map_image,
                Self::thumbnail_size().x(),
                Self::thumbnail_size().y(),
            ),
        }
    }
    pub fn mut_view(
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

    pub fn view(
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
}

pub struct CameraController {
    base_path: String,
    fullsize_map_image: RwLock<FullsizeMapImage>,
    thumbnail_map_image: RwLock<ThumbnailMapImage>,
    request_client: Arc<HTTPClient>,
}

const MAP_BUFFER_PATH: &str = "map.bin";
const SNAPSHOT_FULL_PATH: &str = "snapshot_full.png";
const SNAPSHOT_THUMBNAIL_PATH: &str = "snapshot_thumb.png";

impl CameraController {
    pub fn start(base_path: String, request_client: Arc<HTTPClient>) -> Self {
        let mut fullsize_map_image =
            FullsizeMapImage::open(Path::new(&base_path).join(MAP_BUFFER_PATH));
        let mut thumbnail_map_image = ThumbnailMapImage::from_fullsize(&fullsize_map_image);
        Self {
            fullsize_map_image: RwLock::new(fullsize_map_image),
            thumbnail_map_image: RwLock::new(thumbnail_map_image),
            request_client,
            base_path,
        }
    }

    pub async fn clone_coverage_bitmap(&self) -> BitBox {
        self.fullsize_map_image.read().await.coverage.data.clone()
    }

    #[allow(clippy::cast_sign_loss)]
    fn score_offset(
        decoded_image: &RgbImage,
        base: &FullsizeMapImage,
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
                let map_image_view = base.vec_view(
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

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_possible_wrap)]
    pub async fn shoot_image_to_buffer(
        &self,
        f_cont_locked: Arc<RwLock<FlightComputer>>,
        angle: CameraAngle,
    ) -> Result<Vec2D<u32>, Box<dyn std::error::Error + Send + Sync>> {
        let (position, collected_png) = {
            // TODO: it should be tested if this could be a read lock as well (by not calling update_observation, but current_pos())
            let mut f_cont = f_cont_locked.write().await;
            let ((), collected_png) =
                tokio::join!(f_cont.update_observation(), self.fetch_image_data());
            (f_cont.current_pos(), collected_png)
        };
        let decoded_image = Self::decode_png_data(&collected_png?, angle)?;
        let angle_const = angle.get_square_side_length() / 2;
        let mut offset: Vec2D<i32> = Vec2D::new(
            position.x().round() as i32 - i32::from(angle_const),
            position.y().round() as i32 - i32::from(angle_const),
        )
        .wrap_around_map();

        let mut fullsize_map_image = self.fullsize_map_image.write().await;
        let best_offset = Self::score_offset(
            &decoded_image,
            &fullsize_map_image,
            offset.x() as u32,
            offset.y() as u32,
        );
        offset = (offset + best_offset).wrap_around_map();
        let mut map_image_view = fullsize_map_image.mut_vec_view(offset.cast());
        map_image_view.copy_from(&decoded_image, 0, 0).unwrap();

        let thumbnail_offset = Vec2D::new(
            offset.x() - ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR as i32 * 2,
            offset.y() - ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR as i32 * 2,
        )
        .wrap_around_map()
        .cast();
        let size = u32::from(angle_const) * 2 + ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR * 4;
        let map_image_view = fullsize_map_image.vec_view(thumbnail_offset, Vec2D::new(size, size));

        let resized_image = image::imageops::thumbnail(
            &map_image_view,
            size / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
            size / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
        );
        let mut thumbnail_map_image = self.thumbnail_map_image.write().await;
        thumbnail_map_image
            .mut_view(thumbnail_offset.cast() / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR)
            .copy_from(&resized_image, 0, 0)
            .unwrap();
        fullsize_map_image.coverage.set_region(Vec2D::new(position.x(), position.y()), angle, true);
        Ok(offset.cast())
    }

    async fn fetch_image_data(&self) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
        let response_stream = ShootImageRequest {}.send_request(&self.request_client).await?;

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
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        let mut writer = Cursor::new(Vec::<u8>::new());
        let mut thumbnail_map_image = self.thumbnail_map_image.read().await;
        thumbnail_map_image.image_buffer.write_with_encoder(PngEncoder::new(&mut writer))?;
        Ok(EncodedImageExtract {
            offset: Vec2D::new(0, 0),
            size: Vec2D::map_size() / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
            data: writer.into_inner(),
        })
    }

    #[allow(clippy::cast_sign_loss)]
    pub(crate) async fn export_thumbnail_png(
        &self,
        offset: Vec2D<u32>,
        angle: CameraAngle,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        let offset_vec = offset / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR;
        let size = u32::from(angle.get_square_side_length());
        let size_vec = Vec2D::new(size, size) / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR;

        let mut thumbnail_map_image = self.thumbnail_map_image.read().await;
        let thumbnail = thumbnail_map_image.view(offset_vec, size_vec);

        let mut thumbnail_image = RgbaImage::new(thumbnail.width(), thumbnail.width());
        thumbnail_image.copy_from(&thumbnail, 0, 0).unwrap();
        let mut writer = Cursor::new(Vec::<u8>::new());
        thumbnail_image.write_with_encoder(PngEncoder::new(&mut writer))?;

        Ok(EncodedImageExtract {
            offset: offset_vec.cast(),
            size: size_vec.cast(),
            data: writer.into_inner(),
        })
    }

    #[allow(clippy::cast_sign_loss)]
    pub(crate) async fn export_and_upload_objective_png(
        &self,
        objective_id: usize,
        offset: Vec2D<u32>,
        size: Vec2D<u32>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let map_image = self.fullsize_map_image.read().await;
        let sub_image_view = map_image.vec_view(offset, size);
        let mut sub_image = RgbaImage::new(sub_image_view.width(), sub_image_view.width());
        let mut writer = Cursor::new(Vec::<u8>::new());
        sub_image.write_with_encoder(PngEncoder::new(&mut writer))?;

        ObjectiveImageRequest::new(objective_id, writer.into_inner())
            .send_request(&self.request_client)
            .await?;
        Ok(())
    }

    #[allow(clippy::cast_sign_loss)]
    pub(crate) async fn upload_daily_map_png(&self) -> Result<(), Box<dyn std::error::Error>> {
        DailyMapRequest::new(SNAPSHOT_FULL_PATH)?.send_request(&self.request_client).await?;
        Ok(())
    }

    pub(crate) async fn create_thumb_snapshot(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.thumbnail_map_image
            .read()
            .await
            .image_buffer
            .save(Path::new(&self.base_path).join(SNAPSHOT_THUMBNAIL_PATH))?;
        Ok(())
    }

    pub(crate) async fn create_full_snapshot(&self) -> Result<(), Box<dyn std::error::Error>> {
        println!("[INFO] Exporting Full-View PNG...");
        let start_time = chrono::Utc::now();
        self.fullsize_map_image
            .read()
            .await
            .image_buffer
            .save(Path::new(&self.base_path).join(SNAPSHOT_FULL_PATH))?;
        println!(
            "[INFO] Exported Full-View PNG in {}s!",
            (chrono::Utc::now() - start_time).num_seconds()
        );
        Ok(())
    }

    pub(crate) async fn diff_thumb_snapshot(
        &self,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        if let Ok(mut file) =
            File::open(Path::new(&self.base_path).join(SNAPSHOT_THUMBNAIL_PATH)).await
        {
            let mut old_snapshot_encoded = Vec::<u8>::new();
            file.read_to_end(&mut old_snapshot_encoded).await?;
            let old_snapshot = DynamicImage::from_decoder(PngDecoder::new(&mut Cursor::new(
                old_snapshot_encoded,
            ))?)?
            .to_rgba8();
            let thumbnail_map_image = self.thumbnail_map_image.read().await;
            let mut current_snapshot = thumbnail_map_image.image_buffer.clone();

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
                size: Vec2D::map_size() / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
                data: diff_encoded,
            })
        } else {
            self.export_full_thumbnail_png().await
        }
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub async fn execute_acquisition_cycle(
        self: Arc<Self>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        console_messenger: Arc<ConsoleMessenger>,
        (end_time, last_img_kill): (chrono::DateTime<chrono::Utc>, Arc<Notify>),
        image_max_dt: f32,
        lens: CameraAngle,
        start_index: usize,
    ) -> Vec<(isize, isize)> {
        let mut last_image_flag = false;
        let st = start_index as isize;
        let pic_count = 0;
        let pic_count_lock = Arc::new(Mutex::new(pic_count));
        let mut done_ranges: Vec<(isize, isize)> = Vec::new();
        let overlap = {
            let overlap_dt = (image_max_dt.floor() / 2.0) as isize;
            TimeDelta::seconds(overlap_dt as i64)
        };
        let mut last_mark = (
            st - overlap.num_seconds() as isize,
            chrono::Utc::now() - overlap,
        );
        let mut last_pic = chrono::Utc::now();
        loop {
            let f_cont_lock_clone = Arc::clone(&f_cont_lock);
            let pic_count_lock_clone = Arc::clone(&pic_count_lock);
            let self_clone = Arc::clone(&self);
            let img_init_timestamp = chrono::Utc::now();
            let img_handle = tokio::spawn(async move {
                match self_clone.shoot_image_to_buffer(Arc::clone(&f_cont_lock_clone), lens).await {
                    Ok(offset) => {
                        let pic_num = {
                            let mut lock = pic_count_lock_clone.lock().await;
                            *lock += 1;
                            *lock
                        };
                        println!(
                            "[INFO] Took {pic_num}. picture in cycle at {}. Processed for {}s. Position was {}",
                            img_init_timestamp.format("%d. %H:%M:%S"),
                            (chrono::Utc::now() - img_init_timestamp).num_seconds(),
                            offset
                        );
                        Some(offset)
                    }
                    Err(e) => {
                        println!("[ERROR] Couldn't take picture: {e}");
                        None
                    }
                }
            });

            let mut next_img_due = {
                let next_max_dt = chrono::Utc::now() + TimeDelta::seconds(image_max_dt as i64);
                if next_max_dt > end_time {
                    last_image_flag = true;
                    end_time
                } else {
                    next_max_dt
                }
            };

            let offset = img_handle.await;
            if let Some(off) = offset.ok().flatten() {
                console_messenger.send_thumbnail(off, lens);
                last_pic = img_init_timestamp;
            } else {
                let passed_secs = (last_pic - last_mark.1 + overlap).num_seconds();
                done_ranges.push((last_mark.0, last_mark.0 + passed_secs as isize));
                let tot_passed_secs = (img_init_timestamp - last_mark.1 - overlap).num_seconds();
                last_mark = (tot_passed_secs as isize, chrono::Utc::now());
                next_img_due = chrono::Utc::now() + TimeDelta::seconds(1);
            }

            if last_image_flag {
                let passed_secs = (chrono::Utc::now() - last_mark.1 + overlap).num_seconds();
                done_ranges.push((last_mark.0, last_mark.0 + passed_secs as isize));
                return done_ranges;
            }
            let sleep_time = next_img_due - chrono::Utc::now();
            tokio::select! {
                () = tokio::time::sleep(sleep_time.to_std().unwrap_or(DT_0_STD)) => {},
                () = last_img_kill.notified() => {
                    last_image_flag = true;
                }
            }
        }
    }
}
