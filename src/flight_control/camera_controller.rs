use super::imaging::{
    cycle_state::CycleState,
    map_image::{EncodedImageExtract, FullsizeMapImage, MapImage, OffsetZonedObjectiveImage, ThumbnailMapImage},
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
use crate::mode_control::base_mode::PeriodicImagingEndSignal::{KillLastImage, KillNow};
use crate::{DT_0_STD, error, fatal, info, log, obj};
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;
use futures::StreamExt;
use image::{GenericImageView, ImageReader, Pixel, RgbImage, imageops::Lanczos3};
use std::path::PathBuf;
use std::{
    fs,
    path::Path,
    {io::Cursor, sync::Arc},
};
use tokio::fs::File;
use tokio::io::AsyncWriteExt;
use tokio::sync::{Mutex, RwLock, oneshot};

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
    const LAST_IMG_END_DELAY: TimeDelta = TimeDelta::milliseconds(500);
    const ZO_IMG_FOLDER: &'static str = "zo_img/";

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
        if let Err(e) = fs::create_dir_all(Self::ZO_IMG_FOLDER) {
            fatal!("Failed to create objective image directory: {e}!");
        }
        Self {
            fullsize_map_image: RwLock::new(fullsize_map_image),
            thumbnail_map_image: RwLock::new(thumbnail_map_image),
            request_client,
            base_path,
        }
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
                    .map(
                        |((_, _, existing_pixel), new_pixel)| {
                            if existing_pixel.to_rgb() == new_pixel.to_rgb() { 0 } else { -1 }
                        },
                    )
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

    pub async fn get_image(
        &self,
        f_cont_locked: Arc<RwLock<FlightComputer>>,
        angle: CameraAngle,
    ) -> Result<(Vec2D<I32F32>, Vec2D<i32>, RgbImage), Box<dyn std::error::Error + Send + Sync>>
    {
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
        Ok((position, offset, decoded_image))
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
    pub async fn shoot_image_to_map_buffer(
        &self,
        f_cont_locked: Arc<RwLock<FlightComputer>>,
        angle: CameraAngle,
    ) -> Result<(Vec2D<I32F32>, Vec2D<u32>), Box<dyn std::error::Error + Send + Sync>> {
        let (pos, offset, decoded_image) = self.get_image(f_cont_locked, angle).await?;

        let tot_offset_u32 = {
            let mut fullsize_map_image = self.fullsize_map_image.write().await;
            let best_additional_offset =
                Self::score_offset(&decoded_image, &fullsize_map_image, offset.to_unsigned());
            let tot_offset: Vec2D<u32> =
                (offset + best_additional_offset).wrap_around_map().to_unsigned();
            fullsize_map_image.update_area(tot_offset, &decoded_image);
            tot_offset
        };
        self.update_thumbnail_area_from_fullsize(
            tot_offset_u32,
            u32::from(angle.get_square_side_length() / 2),
        )
        .await;
        Ok((pos, tot_offset_u32))
    }

    pub async fn shoot_image_to_zo_buffer(
        &self,
        f_cont_locked: Arc<RwLock<FlightComputer>>,
        angle: CameraAngle,
        zoned_objective_map_image: Option<&mut OffsetZonedObjectiveImage>,
    ) -> Result<Vec2D<I32F32>, Box<dyn std::error::Error + Send + Sync>> {
        let (pos, offset, decoded_image) = self.get_image(f_cont_locked, angle).await?;
        let offset_u32 = offset.to_unsigned();
        if let Some(image) = zoned_objective_map_image {
            image.update_area(offset_u32, &decoded_image);
        }
        
        Ok(pos)
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
            &resized_image,
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
        export_path: Option<PathBuf>,
        zoned_objective_map_image: Option<&OffsetZonedObjectiveImage>
    ) -> Result<(), Box<dyn std::error::Error>> {
        let encoded_image = if let Some(zoned_objective_map_image) = zoned_objective_map_image {
            zoned_objective_map_image.export_as_png()?
        } else {
            let map_image = self.fullsize_map_image.read().await;
            map_image.export_area_as_png(offset, size)?
        };
        if let Some(img_path) = export_path {
            let mut img_file = File::create(&img_path).await?;
            img_file.write_all(encoded_image.data.as_slice()).await?;
            ObjectiveImageRequest::new(objective_id, img_path)
                .send_request(&self.request_client)
                .await?;
        }
        
        Ok(())
    }

    pub(crate) fn generate_zo_img_path(id: usize) -> PathBuf {
        let dir = Path::new(Self::ZO_IMG_FOLDER);
        let mut path = dir.join(format!("zo_{id}.png"));
        let mut counter = 0;
        while path.exists() {
            path = dir.join(format!("zo_{id}_{counter}.png",));
            counter += 1;
        }
        path
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
    pub(crate) async fn export_full_snapshot(&self) -> Result<(), Box<dyn std::error::Error>> {
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
        self: &Arc<Self>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        console_messenger: Arc<ConsoleMessenger>,
        (end_time, kill): (DateTime<Utc>, oneshot::Receiver<PeriodicImagingEndSignal>),
        image_max_dt: I32F32,
        start_index: usize,
    ) -> Vec<(isize, isize)> {
        log!(
            "Starting acquisition cycle. Deadline: {}",
            end_time.format("%H:%M:%S")
        );
        let lens = f_cont_lock.read().await.current_angle();
        let mut kill_box = Box::pin(kill);
        let mut last_image_flag = false;

        let pic_count_lock = Arc::new(Mutex::new(0));
        let mut state = CycleState::init_cycle(image_max_dt, start_index as isize);

        loop {
            let (img_t, offset) =
                Self::exec_map_capture(self, &f_cont_lock, &pic_count_lock, lens).await;

            let mut next_img_due = Self::get_next_map_img(image_max_dt, end_time);
            if let Some(off) = offset {
                console_messenger.send_thumbnail(off, lens);
                state.update_success(img_t);
            } else {
                state.update_failed(img_t);
                error!("Rescheduling failed picture immediately!");
                next_img_due = Utc::now() + TimeDelta::seconds(1);
            }

            if last_image_flag {
                return state.finish();
            } else if next_img_due + Self::LAST_IMG_END_DELAY >= end_time {
                last_image_flag = true;
            }

            let sleep_time = next_img_due - Utc::now();
            tokio::select! {
                () = tokio::time::sleep(sleep_time.to_std().unwrap_or(DT_0_STD)) => {},
                msg = &mut kill_box => {
                     match msg.unwrap_or_else(|e| {
                            error!("Couldn't receive kill signal: {e}");
                            KillNow
                        }) {
                        KillLastImage => last_image_flag = true,
                        KillNow => {
                             return state.finish();
                        }
                    }
                }
            }
        }
    }

    pub async fn execute_zo_target_cycle(
        self: Arc<Self>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        deadline: DateTime<Utc>,
        zoned_objective_image_buffer: &mut Option<OffsetZonedObjectiveImage>,
        offset: Vec2D<u32>, dimensions: Vec2D<u32>
    ) {
        obj!("Starting acquisition cycle for objective!");
        zoned_objective_image_buffer.replace(OffsetZonedObjectiveImage::new(offset, dimensions));
        let lens = f_cont_lock.read().await.current_angle();
        let mut pics = 0;
        let deadline_cont = deadline - Utc::now() > TimeDelta::seconds(20);
        let step_print = if deadline_cont { 10 } else { 2 };
        loop {
            let next_img_due = Utc::now() + TimeDelta::seconds(1);
            let img_init_timestamp = Utc::now();
            match self.shoot_image_to_zo_buffer(Arc::clone(&f_cont_lock), lens, zoned_objective_image_buffer.as_mut()).await {
                Ok(pos) => {
                    pics += 1;
                    let s = (Utc::now() - img_init_timestamp).num_seconds();
                    if pics % step_print == 0 {
                        obj!("Took {pics:02}. picture. Processed for {s}s. Position was {pos}");
                    }
                }
                Err(e) => {
                    error!("Couldn't take picture: {e}");
                }
            }
            if Utc::now() > deadline {
                return;
            }
            tokio::time::sleep((next_img_due - Utc::now()).to_std().unwrap_or(DT_0_STD)).await;
        }
    }

    fn get_next_map_img(img_max_dt: I32F32, end_time: DateTime<Utc>) -> DateTime<Utc> {
        let next_max_dt = Utc::now() + TimeDelta::seconds(img_max_dt.to_num::<i64>());
        if next_max_dt > end_time { end_time - Self::LAST_IMG_END_DELAY } else { next_max_dt }
    }

    async fn exec_map_capture(
        self: &Arc<Self>,
        f_cont: &Arc<RwLock<FlightComputer>>,
        p_c: &Arc<Mutex<i32>>,
        lens: CameraAngle,
    ) -> (DateTime<Utc>, Option<Vec2D<u32>>) {
        let f_cont_clone = Arc::clone(f_cont);
        let p_c_clone = Arc::clone(p_c);
        let self_clone = Arc::clone(self);
        let img_init_timestamp = Utc::now();

        let img_handle = tokio::spawn(async move {
            match self_clone.shoot_image_to_map_buffer(Arc::clone(&f_cont_clone), lens).await {
                Ok((pos, offset)) => {
                    let pic_num = {
                        let mut lock = p_c_clone.lock().await;
                        *lock += 1;
                        *lock
                    };
                    let s = (Utc::now() - img_init_timestamp).num_seconds();
                    info!("Took {pic_num:02}. picture. Processed for {s}s. Position was {pos}");
                    Some(offset)
                }
                Err(e) => {
                    error!("Couldn't take picture: {e}");
                    None
                }
            }
        });

        let res = img_handle.await.ok().flatten();
        (img_init_timestamp, res)
    }
}
