use crate::flight_control::camera_controller::CameraController;
use crate::flight_control::objective::beacon_objective::BeaconObjective;
use crate::flight_control::{
    common::vec2d::Vec2D, flight_computer::FlightComputer, flight_state::FlightState,
    objective::known_img_objective::KnownImgObjective,
};
use crate::http_handler::{
    ZoneType,
    http_request::{
        objective_list_get::ObjectiveListRequest, request_common::NoBodyHTTPRequestType,
    },
};
use crate::{error, event, fatal, info, log, warn, DT_0_STD};
use chrono::{DateTime, NaiveTime, TimeDelta, TimeZone, Utc};
use csv::Writer;
use fixed::types::I32F32;
use futures::StreamExt;
use reqwest_eventsource::{Event, EventSource};
use std::{collections::HashSet, env, sync::Arc, time::Duration};
use tokio::{
    sync::{Notify, RwLock, broadcast, mpsc, mpsc::Receiver},
    time::Instant,
};

pub struct Supervisor {
    f_cont_lock: Arc<RwLock<FlightComputer>>,
    safe_mon: Arc<Notify>,
    zo_mon: mpsc::Sender<KnownImgObjective>,
    bo_mon: mpsc::Sender<BeaconObjective>,
    event_hub: broadcast::Sender<(DateTime<Utc>, String)>,
}

impl Supervisor {
    /// Constant update interval for observation updates in the `run()` method
    const OBS_UPDATE_INTERVAL: Duration = Duration::from_millis(500);
    /// Constant update interval for objective updates in the `run()` method
    const OBJ_UPDATE_INTERVAL: TimeDelta = TimeDelta::seconds(15);
    /// Constant minimum time delta to the objective start for sending the objective to `main`
    const B_O_MIN_DT: TimeDelta = TimeDelta::minutes(20);
    const TRACK_POS_ENV: &'static str = "TRACK_MELVIN_POS";

    /// Creates a new instance of `Supervisor`
    pub fn new(
        f_cont_lock: Arc<RwLock<FlightComputer>>,
    ) -> (
        Supervisor,
        Receiver<KnownImgObjective>,
        Receiver<BeaconObjective>,
    ) {
        let (tx_obj, rx_obj) = mpsc::channel(10);
        let (tx_beac, rx_beac) = mpsc::channel(10);
        let (event_send, _) = broadcast::channel(10);
        (
            Self {
                f_cont_lock,
                safe_mon: Arc::new(Notify::new()),
                zo_mon: tx_obj,
                bo_mon: tx_beac,
                event_hub: event_send,
            },
            rx_obj,
            rx_beac,
        )
    }

    pub fn safe_mon(&self) -> Arc<Notify> { Arc::clone(&self.safe_mon) }

    pub fn subscribe_event_hub(&self) -> broadcast::Receiver<(DateTime<Utc>, String)> {
        self.event_hub.subscribe()
    }

    pub async fn run_announcement_hub(&self) {
        let url = {
            let client = self.f_cont_lock.read().await.client();
            client.url().to_string()
        };
        let mut es = EventSource::get(url + "/announcements");
        while let Some(event) = es.next().await {
            match event {
                Ok(Event::Open) => log!("Starting event supervisor loop!"),
                Ok(Event::Message(msg)) => {
                    let msg_str = format!("{msg:#?}");
                    if self.event_hub.send((Utc::now(), msg_str)).is_err() {
                        event!("No Receiver for: {msg:#?}");
                    }
                }
                Err(err) => {
                    error!("EventSource error: {err}");
                    es.close();
                }
            }
        }
        fatal!("EventSource disconnected!");
    }

    pub async fn run_daily_map_uploader(&self, c_cont: Arc<CameraController>) {
        let now = Utc::now();
        let end_of_day = NaiveTime::from_hms_opt(23, 59, 55).unwrap();
        let upload_t = now.date_naive().and_time(end_of_day);
        let mut next_upload_t = Utc.from_utc_datetime(&upload_t);
        loop {
            let next_upload_dt =
                (next_upload_t - Utc::now()).to_std().unwrap_or(DT_0_STD);
            tokio::time::sleep(next_upload_dt).await;
            c_cont.export_full_snapshot().await.unwrap_or_else(|e| {
                error!("Error exporting full snapshot: {e}.");
            });
            c_cont.upload_daily_map_png().await.unwrap_or_else(|e| {
                error!("Error uploading Daily Map: {e}.");
            });
            info!("Successfully uploaded Daily Map!");
            next_upload_t = next_upload_t.checked_add_signed(TimeDelta::days(1)).unwrap();
        }
    }

    /// Starts the supervisor loop to periodically call `update_observation`
    /// and monitor position & state deviations.
    #[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
    pub async fn run_obs_obj_mon(&self) {
        // let mut next_safe = start + TimeDelta::seconds(500);
        let mut pos_csv = if env::var(Self::TRACK_POS_ENV).is_ok() {
            log!("Activated position tracking!");
            Some(Writer::from_writer(
                std::fs::OpenOptions::new()
                    .create(true)
                    .write(true)
                    .truncate(true)
                    .open("pos.csv")
                    .ok()
                    .unwrap(),
            ))
        } else {
            None
        };
        // **********
        // TODO: pos monitoring in kalman filter + listening for reset_pos events
        let mut last_pos: Option<Vec2D<I32F32>> = None;
        let mut last_timestamp = Utc::now();
        let mut last_vel = self.f_cont_lock.read().await.current_vel();
        let mut last_objective_check = Utc::now() - Self::OBJ_UPDATE_INTERVAL;
        let mut id_list: HashSet<usize> = HashSet::new();
        log!("Starting obs/obj supervisor loop!");
        loop {
            let mut f_cont = self.f_cont_lock.write().await;
            // Update observation and fetch new position
            f_cont.update_observation().await;
            let last_update = Instant::now();
            /*
            if Utc::now() > next_safe {
                let act_state = f_cont.state();
                f_cont.one_time_safe();
                next_safe += TimeDelta::seconds(5000);
                log!("One time safe mode activated Actual State was {act_state}!");
            }*/

            let current_pos = f_cont.current_pos();
            let current_vel = f_cont.current_vel();
            let is_safe_trans = {
                let current_state = f_cont.state();
                let target_state = f_cont.target_state();
                current_state == FlightState::Transition && target_state.is_none()
            };

            drop(f_cont); // Release the lock early to avoid blocking

            if let Some(previous_pos) = last_pos {
                let dt = Utc::now() - last_timestamp;
                let dt_secs = dt.num_milliseconds() as f32 / 1000.0;

                let expected_pos = previous_pos + last_vel * I32F32::from_num(dt_secs);

                let expected_pos_wrapped = expected_pos.wrap_around_map();
                // TODO: this diff value should also consider wrapping (if actual pos wrapped, but expected didnt)
                let diff = current_pos - expected_pos_wrapped;
                if let Some(write) = pos_csv.as_mut() {
                    write
                        .write_record(&[
                            current_pos.x().to_string(),
                            current_pos.y().to_string(),
                            expected_pos_wrapped.x().to_string(),
                            expected_pos_wrapped.y().to_string(),
                            diff.x().to_string(),
                            diff.y().to_string(),
                        ])
                        .expect("[FATAL] Could not write to csv file!");
                }

                // check safe mode transition
                if is_safe_trans {
                    warn!("Unplanned Safe Mode Transition Detected! Notifying!");
                    self.safe_mon.notify_one();
                    self.f_cont_lock.write().await.safe_detected();
                }
            }

            last_pos = Some(current_pos);
            last_timestamp = Utc::now();
            last_vel = current_vel;

            if last_objective_check + Self::OBJ_UPDATE_INTERVAL < Utc::now() {
                let handle = self.f_cont_lock.read().await.client();
                let objective_list = ObjectiveListRequest {}.send_request(&handle).await.unwrap();
                let mut send_img_objs = Vec::new();
                let mut send_beac_objs = Vec::new();

                for img_obj in objective_list.img_objectives() {
                    let obj_on = img_obj.start() < Utc::now() && img_obj.end() > Utc::now();
                    let is_secret = matches!(img_obj.zone_type(), ZoneType::SecretZone(_));
                    if obj_on && !id_list.contains(&img_obj.id()) && !is_secret {
                        send_img_objs.push(KnownImgObjective::try_from(img_obj.clone()).unwrap());
                    }
                }
                for b_o in objective_list.beacon_objectives() {
                    let obj_on = b_o.start() < Utc::now() && b_o.end() > Utc::now();
                    if obj_on && !id_list.contains(&b_o.id()) {
                        send_beac_objs.push(BeaconObjective::from(b_o.clone()));
                    }
                }
                for obj in send_img_objs {
                    id_list.insert(obj.id());
                    self.zo_mon.send(obj).await.unwrap();
                }
                for beac_obj in send_beac_objs {
                    id_list.insert(beac_obj.id());
                    self.bo_mon.send(beac_obj).await.unwrap();
                }
                last_objective_check = Utc::now();
            }

            tokio::time::sleep_until(last_update + Self::OBS_UPDATE_INTERVAL).await;
        }
    }
}
