use super::imaging::map_image::{
    EncodedImageExtract, FullsizeMapImage, MapImage, ThumbnailMapImage,
};
use crate::console_communication::ConsoleMessenger;
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
use crate::mode_control::base_mode::PeriodicImagingEndSignal;
use crate::{error, info, DT_0_STD};
use bitvec::boxed::BitBox;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;
use futures::StreamExt;
use image::{imageops::Lanczos3, GenericImageView, ImageReader, Pixel, RgbImage};
use std::{
    path::Path,
    {io::Cursor, sync::Arc},
};
use tokio::sync::{oneshot, Mutex, RwLock};

/// A struct for managing camera-related operations and map snapshots.
pub struct CameraController {
    /// The base path for saving map image data.
    base_path: String,
    /// The lock-protected full-size map image.
    fullsize_map_image: RwLock<FullsizeMapImage>,
    /// The lock-protected thumbnail map image.
    thumbnail_map_image: RwLock<ThumbnailMapImage>,
    /// The HTTP client for sending requests.
    request_client: Arc<HTTPClient>,
}

/// Path to the binary map buffer file.
const MAP_BUFFER_PATH: &str = "map.bin";
/// Path to the full-size snapshot file.
const SNAPSHOT_FULL_PATH: &str = "snapshot_full.png";
/// Path to the thumbnail snapshot file.
const SNAPSHOT_THUMBNAIL_PATH: &str = "snapshot_thumb.png";

impl CameraController {
    /// Initializes the `CameraController` with the given base path and HTTP client.
    ///
    /// # Arguments
    ///
    /// * `base_path` - The base path for storing files.
    /// * `request_client` - The HTTP client for sending requests.
    ///
    /// # Returns
    ///
    /// A new instance of `CameraController`.
    pub fn start(base_path: String, request_client: Arc<HTTPClient>) -> Self {
        let fullsize_map_image =
            FullsizeMapImage::open(Path::new(&base_path).join(MAP_BUFFER_PATH));
        let thumbnail_map_image =
            ThumbnailMapImage::from_snapshot(Path::new(&base_path).join(SNAPSHOT_THUMBNAIL_PATH));
        Self {
            fullsize_map_image: RwLock::new(fullsize_map_image),
            thumbnail_map_image: RwLock::new(thumbnail_map_image),
            request_client,
            base_path,
        }
    }

    /// Clones the coverage bitmap of the full-size map image.
    ///
    /// # Returns
    ///
    /// A copy of the coverage bitmap as a `BitBox`.
    pub async fn clone_coverage_bitmap(&self) -> BitBox {
        self.fullsize_map_image.read().await.coverage.data.clone()
    }

    /// Scores the offset by comparing the decoded image against the map base image.
    ///
    /// # Arguments
    ///
    /// * `decoded_image` - The decoded image to match.
    /// * `base` - The reference full-size map image.
    /// * `offset` - The initial offset to evaluate.
    ///
    /// # Returns
    ///
    /// The best scored offset as `Vec2D<i32>`.
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

    /// Captures an image, processes it, and stores it in the map buffer.
    ///
    /// # Arguments
    ///
    /// * `f_cont_locked` - The lock-protected flight computer.
    /// * `angle` - The camera angle and field of view.
    ///
    /// # Returns
    ///
    /// The updated offset as `Vec2D<u32>` or an error.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_possible_wrap)]
    pub async fn shoot_image_to_buffer(
        &self,
        f_cont_locked: Arc<RwLock<FlightComputer>>,
        angle: CameraAngle,
    ) -> Result<Vec2D<u32>, Box<dyn std::error::Error + Send + Sync>> {
        let (position, collected_png) = {
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

    /// Updates the thumbnail area of the map based on the full-size map data.
    ///
    /// # Arguments
    ///
    /// * `offset` - Offset to update.
    /// * `size` - Size of the region to update.
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

    /// Fetches image data from the camera as a byte vector.
    ///
    /// # Returns
    ///
    /// The raw PNG data or an error.
    async fn fetch_image_data(&self) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
        let response_stream = ShootImageRequest {}.send_request(&self.request_client).await?;

        let mut collected_png: Vec<u8> = Vec::new();
        futures::pin_mut!(response_stream);

        while let Some(Ok(chunk_result)) = response_stream.next().await {
            collected_png.extend_from_slice(&chunk_result[..]);
        }

        Ok(collected_png)
    }

    /// Decodes PNG data into an RGB image and resizes it based on the camera angle.
    ///
    /// # Arguments
    ///
    /// * `collected_png` - Raw PNG data.
    /// * `angle` - The camera angle defining the image resolution.
    ///
    /// # Returns
    ///
    /// The decoded and resized image as `RgbImage` or an error.
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

    /// Exports a specific region of the map as a PNG and uploads it to the server associated with the given objective ID.
    ///
    /// # Arguments
    ///
    /// * `objective_id` - The identifier of the objective to associate the exported PNG with.
    /// * `offset` - The offset in the map to start the export.
    /// * `size` - The dimensions of the region to export as a PNG.
    ///
    /// # Returns
    ///
    /// A result indicating the success or failure of the operation.
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

    /// Uploads the daily map snapshot as a PNG to the server.
    ///
    /// # Returns
    ///
    /// A result indicating the success or failure of the operation.
    #[allow(clippy::cast_sign_loss)]
    pub(crate) async fn upload_daily_map_png(&self) -> Result<(), Box<dyn std::error::Error>> {
        DailyMapRequest::new(SNAPSHOT_FULL_PATH)?.send_request(&self.request_client).await?;
        Ok(())
    }

    /// Creates and saves a thumbnail snapshot of the map.
    ///
    /// # Returns
    ///
    /// A result indicating the success or failure of the operation.
    pub(crate) async fn create_thumb_snapshot(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.thumbnail_map_image
            .read()
            .await
            .create_snapshot(Path::new(&self.base_path).join(SNAPSHOT_THUMBNAIL_PATH))
    }

    /// Creates and saves a full-size snapshot of the map.
    ///
    /// # Returns
    ///
    /// A result indicating the success or failure of the operation.
    pub(crate) async fn create_full_snapshot(&self) -> Result<(), Box<dyn std::error::Error>> {
        let start_time = Utc::now();
        self.fullsize_map_image
            .read()
            .await
            .create_snapshot(Path::new(&self.base_path).join(SNAPSHOT_FULL_PATH))?;
        info!(
            "Exported Full-View PNG in {}s!",
            (Utc::now() - start_time).num_seconds()
        );
        Ok(())
    }

    /// Exports a part of the thumbnail map as a PNG.
    ///
    /// # Arguments
    ///
    /// * `offset` - The offset to start exporting from.
    /// * `angle` - The camera angle, which influences the resolution and dimensions.
    ///
    /// # Returns
    ///
    /// A result containing the extracted PNG image data or an error.
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

    /// Exports the entire map thumbnail as a PNG.
    ///
    /// # Returns
    ///
    /// A result containing the extracted PNG image data or an error.
    pub(crate) async fn export_full_thumbnail_png(
        &self,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        self.thumbnail_map_image.read().await.export_as_png()
    }

    /// Compares the thumbnail map with its saved snapshot.
    ///
    /// # Returns
    ///
    /// A result containing the difference as an encoded PNG image or an error.
    pub(crate) async fn diff_thumb_snapshot(
        &self,
    ) -> Result<EncodedImageExtract, Box<dyn std::error::Error>> {
        self.thumbnail_map_image
            .read()
            .await
            .diff_with_snapshot(Path::new(&self.base_path).join(SNAPSHOT_THUMBNAIL_PATH))
            .await
    }

    /// Executes a series of image acquisitions, processes them, and updates the associated map buffers.
    ///
    /// # Arguments
    ///
    /// * `f_cont_lock` - Lock-protected flight computer controlling the acquisition cycle.
    /// * `console_messenger` - Used for sending notifications during processing.
    /// * `(end_time, last_img_kill)` - The end time for the cycle and a notify object to terminate the process prematurely.
    /// * `image_max_dt` - Maximum allowed interval between consecutive images.
    /// * `lens` - The camera angle and field of view.
    /// * `start_index` - The starting index for tracking image acquisitions.
    ///
    /// # Returns
    ///
    /// A vector of completed (start, end) time ranges when images were successfully taken.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_possible_wrap)]
    pub async fn execute_acquisition_cycle(
        self: Arc<Self>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        console_messenger: Arc<ConsoleMessenger>,
        (end_time, kill): (DateTime<Utc>, oneshot::Receiver<PeriodicImagingEndSignal>),
        image_max_dt: I32F32,
        lens: CameraAngle,
        start_index: usize,
    ) -> Vec<(isize, isize)> {
        fn get_p_secs(
            last_pic: Option<DateTime<Utc>>,
            last_mark: DateTime<Utc>,
            overlap: TimeDelta,
        ) -> i64 {
            if let Some(last_pic_val) = last_pic {
                (last_pic_val - last_mark + overlap).num_seconds()
            } else {
                0
            }
        }
        let mut kill_box = Box::pin(kill);
        let mut last_image_flag = false;
        let st = start_index as isize;
        let pic_count = 0;
        let pic_count_lock = Arc::new(Mutex::new(pic_count));
        let mut done_ranges: Vec<(isize, isize)> = Vec::new();
        let overlap = {
            let overlap_dt = (image_max_dt.floor() / I32F32::lit("2.0")).to_num::<isize>();
            TimeDelta::seconds(overlap_dt as i64)
        };
        let mut last_mark = (st - overlap.num_seconds() as isize, Utc::now() - overlap);
        let mut last_pic = None;
        loop {
            let f_cont_lock_clone = Arc::clone(&f_cont_lock);
            let pic_count_lock_clone = Arc::clone(&pic_count_lock);
            let self_clone = Arc::clone(&self);
            let img_init_timestamp = Utc::now();
            let img_handle = tokio::spawn(async move {
                match self_clone.shoot_image_to_buffer(Arc::clone(&f_cont_lock_clone), lens).await {
                    Ok(offset) => {
                        let pic_num = {
                            let mut lock = pic_count_lock_clone.lock().await;
                            *lock += 1;
                            *lock
                        };
                        info!("Took {pic_num:02}. picture in cycle at {}. Processed for {}s. Position was {}",
                            img_init_timestamp.format("%d. %H:%M:%S"),
                            (Utc::now() - img_init_timestamp).num_seconds(),
                            offset
                        );
                        Some(offset)
                    }
                    Err(e) => {
                        error!("Couldn't take picture: {e}");
                        None
                    }
                }
            });

            let mut next_img_due = {
                let next_max_dt = Utc::now() + TimeDelta::seconds(image_max_dt.to_num::<i64>());
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
                last_pic = Some(img_init_timestamp);
            } else {
                let p_secs = get_p_secs(last_pic, last_mark.1, overlap);
                done_ranges.push((last_mark.0, last_mark.0 + p_secs as isize));
                let tot_passed_secs = (img_init_timestamp - last_mark.1 - overlap).num_seconds();
                last_mark = (tot_passed_secs as isize, Utc::now());
                last_pic = None;
                next_img_due = Utc::now() + TimeDelta::seconds(1);
            }

            if last_image_flag {
                let p_secs = get_p_secs(last_pic, last_mark.1, overlap);
                done_ranges.push((last_mark.0, last_mark.0 + p_secs as isize));
                return done_ranges;
            }
            let sleep_time = next_img_due - Utc::now();
            tokio::select! {
                () = tokio::time::sleep(sleep_time.to_std().unwrap_or(DT_0_STD)) => {},
                msg = &mut kill_box => {
                    let msg_unwr = msg.unwrap_or_else(
                        |e| {
                            error!("Couldn't receive kill signal: {e}");
                            PeriodicImagingEndSignal::KillNow
                        });
                    match msg_unwr{
                        PeriodicImagingEndSignal::KillLastImage => last_image_flag = true,
                        PeriodicImagingEndSignal::KillNow => {
                             let p_secs = get_p_secs(last_pic, last_mark.1, overlap);
                            done_ranges.push((last_mark.0, last_mark.0 + p_secs as isize));
                            return done_ranges
                        }
                    }
                }
            }
        }
    }
}
