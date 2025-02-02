use crate::console_communication::console_messenger::ConsoleMessenger;
use crate::flight_control::{
    camera_state::CameraAngle, common::vec2d::Vec2D, flight_computer::FlightComputer,
};
use crate::http_handler::{
    http_client::HTTPClient,
    http_request::{
        daily_map_post::DailyMapRequest,
        objective_image_post::ObjectiveImageRequest,
        request_common::{MultipartBodyHTTPRequestType, NoBodyHTTPRequestType},
        shoot_image_get::ShootImageRequest,
    },
};
use crate::DT_0_STD;
use bitvec::boxed::BitBox;
use chrono::TimeDelta;
use fixed::types::I32F32;
use futures::StreamExt;
use image::{imageops::Lanczos3, ImageReader, RgbImage};
use image::{GenericImageView, Pixel};
use num::ToPrimitive;
use std::{
    path::Path,
    {io::Cursor, sync::Arc},
};
use tokio::sync::{Mutex, Notify, RwLock};

use super::imaging::map_image::{
    EncodedImageExtract, FullsizeMapImage, MapImage, ThumbnailMapImage,
};

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
        let fullsize_map_image =
            FullsizeMapImage::open(Path::new(&base_path).join(MAP_BUFFER_PATH));
        //let thumbnail_map_image = ThumbnailMapImage::from_fullsize(&fullsize_map_image);
        let thumbnail_map_image =
            ThumbnailMapImage::from_snapshot(Path::new(&base_path).join(SNAPSHOT_THUMBNAIL_PATH));
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

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_wrap)]
    fn score_offset(
        decoded_image: &RgbImage,
        base: &FullsizeMapImage,
        offset: Vec2D<u32>,
    ) -> Vec2D<i32> {
        let mut best_score = i32::MIN;
        let mut best_additional_offset = Vec2D::new(0, 0);
        for additional_offset_x in -2..=2 {
            for additional_offset_y in -2..=2 {
                let current_offset: Vec2D<u32> = Vec2D::new(
                    offset.x() as i32 + additional_offset_x,
                    offset.y() as i32 + additional_offset_y,
                )
                .wrap_around_map()
                .to_unsigned();
                let map_image_view = base.vec_view(
                    current_offset,
                    Vec2D::new(decoded_image.width(), decoded_image.height()),
                );
                let mut score: i32 = map_image_view
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

                score -= additional_offset_x.abs() + additional_offset_y.abs();
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
        let offset: Vec2D<i32> = Vec2D::new(
            position.x().round().to_num::<i32>() - i32::from(angle_const),
            position.y().round().to_num::<i32>() - i32::from(angle_const),
        )
        .wrap_around_map();

        let tot_offset_u32 = {
            let mut fullsize_map_image = self.fullsize_map_image.write().await;
            let best_additional_offset =
                Self::score_offset(&decoded_image, &fullsize_map_image, offset.to_unsigned());
            let tot_offset: Vec2D<u32> =
                (offset + best_additional_offset).wrap_around_map().to_unsigned();
            fullsize_map_image.update_area(tot_offset, decoded_image);

            fullsize_map_image.coverage.set_region(
                Vec2D::new(position.x(), position.y()),
                angle,
                true,
            );
            tot_offset
        };
        self.update_thumbnail_area_from_fullsize(tot_offset_u32, u32::from(angle_const)).await;
        Ok(tot_offset_u32)
    }

    #[allow(clippy::cast_possible_wrap)]
    async fn update_thumbnail_area_from_fullsize(&self, offset: Vec2D<u32>, size: u32) {
        let thumbnail_offset = Vec2D::new(
            offset.x() as i32 - ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR as i32 * 2,
            offset.y() as i32 - ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR as i32 * 2,
        )
        .wrap_around_map()
        .to_unsigned();
        let size_scaled = size * 2 + ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR * 4;
        let fullsize_map_image = self.fullsize_map_image.read().await;
        let map_image_view =
            fullsize_map_image.vec_view(thumbnail_offset, Vec2D::new(size_scaled, size_scaled));

        let resized_image = image::imageops::thumbnail(
            &map_image_view,
            size_scaled / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
            size_scaled / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
        );
        self.thumbnail_map_image.write().await.update_area(
            thumbnail_offset / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
            resized_image,
        );
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

    #[allow(clippy::cast_sign_loss)]
    pub(crate) async fn export_and_upload_objective_png(
        &self,
        objective_id: usize,
        offset: Vec2D<u32>,
        size: Vec2D<u32>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let map_image = self.fullsize_map_image.read().await;
        let encoded_image = map_image.export_area_as_png(offset, size)?;

        ObjectiveImageRequest::new(objective_id, encoded_image.data)
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
            .create_snapshot(Path::new(&self.base_path).join(SNAPSHOT_THUMBNAIL_PATH))
    }
    pub(crate) async fn create_full_snapshot(&self) -> Result<(), Box<dyn std::error::Error>> {
        println!("[INFO] Exporting Full-View PNG...");
        let start_time = chrono::Utc::now();
        self.fullsize_map_image
            .read()
            .await
            .create_snapshot(Path::new(&self.base_path).join(SNAPSHOT_FULL_PATH))?;
        println!(
            "[INFO] Exported Full-View PNG in {}s!",
            (chrono::Utc::now() - start_time).num_seconds()
        );
        Ok(())
    }

    pub(crate) async fn export_thumbnail_png(
        &self,
        offset: Vec2D<u32>,
        angle: CameraAngle,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        let size =
            u32::from(angle.get_square_side_length()) / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR;
        self.thumbnail_map_image.read().await.export_area_as_png(
            offset / ThumbnailMapImage::THUMBNAIL_SCALE_FACTOR,
            Vec2D::new(size, size),
        )
    }
    pub(crate) async fn export_full_thumbnail_png(
        &self,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        self.thumbnail_map_image.read().await.export_as_png()
    }

    pub(crate) async fn diff_thumb_snapshot(
        &self,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        self.thumbnail_map_image
            .read()
            .await
            .diff_with_snapshot(Path::new(&self.base_path).join(SNAPSHOT_THUMBNAIL_PATH))
            .await
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_possible_wrap)]
    pub async fn execute_acquisition_cycle(
        self: Arc<Self>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        console_messenger: Arc<ConsoleMessenger>,
        (end_time, last_img_kill): (chrono::DateTime<chrono::Utc>, Arc<Notify>),
        image_max_dt: I32F32,
        lens: CameraAngle,
        start_index: usize,
    ) -> Vec<(isize, isize)> {
        let mut last_image_flag = false;
        let st = start_index as isize;
        let pic_count = 0;
        let pic_count_lock = Arc::new(Mutex::new(pic_count));
        let mut done_ranges: Vec<(isize, isize)> = Vec::new();
        let overlap = {
            let overlap_dt = (image_max_dt.floor() / I32F32::lit("2.0")).to_isize().unwrap();
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
                let next_max_dt =
                    chrono::Utc::now() + TimeDelta::seconds(image_max_dt.to_i64().unwrap());
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
