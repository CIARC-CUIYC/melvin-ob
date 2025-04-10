use super::{FlightComputer, FlightState};
use crate::imaging::CameraController;
use crate::objective::{BeaconObjective, KnownImgObjective};
use crate::http_handler::{
    ZoneType, ImageObjective,
    http_request::{
        objective_list_get::ObjectiveListRequest, request_common::NoBodyHTTPRequestType,
    },
};
use crate::{DT_0_STD, error, event, fatal, info, log, warn, obj};
use chrono::{DateTime, NaiveTime, TimeDelta, TimeZone, Utc};
use futures::StreamExt;
use reqwest_eventsource::{Event, EventSource};
use std::{collections::HashSet, env, sync::Arc, time::Duration};
use tokio::{
    sync::{Notify, RwLock, broadcast, mpsc, mpsc::Receiver},
    time::Instant,
};

/// The [`Supervisor`] is responsible for high-level management of active operations,
/// including observation tracking, secret objective handling, daily map uploads,
/// safe-mode monitoring, and real-time event listening.
///
/// It acts as coordinator between observation, event streams, and
/// objective scheduling, while ensuring asynchronous notifications and channel-based
/// communication between system components.
pub struct Supervisor {
    /// Lock-protected reference to the [`FlightComputer`], used for updating the global observation.
    f_cont_lock: Arc<RwLock<FlightComputer>>,
    /// Notifier that signals when a safe-mode transition is detected.
    safe_mon: Arc<Notify>,
    /// Channel for sending newly discovered zoned objectives to the main scheduling system.
    zo_mon: mpsc::Sender<KnownImgObjective>,
    /// Channel for sending active beacon objectives to the main scheduling system.
    bo_mon: mpsc::Sender<BeaconObjective>,
    /// Broadcast channel for relaying real-time mission announcements or telemetry updates.
    event_hub: broadcast::Sender<(DateTime<Utc>, String)>,
    /// In-memory buffer of currently known secret imaging objectives that await triggering.
    current_secret_objectives: RwLock<Vec<ImageObjective>>,
}

impl Supervisor {
    /// Constant update interval for observation updates in the `run()` method
    const OBS_UPDATE_INTERVAL: Duration = Duration::from_millis(500);
    /// Constant update interval for objective updates in the `run()` method
    const OBJ_UPDATE_INTERVAL: TimeDelta = TimeDelta::seconds(15);
    /// Constant minimum time delta to the objective start for sending the objective to `main`
    const B_O_MIN_DT: TimeDelta = TimeDelta::minutes(20);
    /// Environment variable used to skip known objectives by ID (comma-separated).
    const ENV_SKIP_OBJ: &'static str = "SKIP_OBJ";

    /// Creates a new [`Supervisor`] instance and returns associated receivers
    /// for zoned and beacon objectives.
    ///
    /// # Arguments
    /// * `f_cont_lock` – Shared lock to the flight computer state.
    ///
    /// # Returns
    /// Tuple of ([`Supervisor`], `zo_receiver`, `bo_receiver`)
    pub(crate) fn new(
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
                current_secret_objectives: RwLock::new(vec![]),
            },
            rx_obj,
            rx_beac,
        )
    }

    /// Returns a clone of the safe-mode notifier.
    pub(crate) fn safe_mon(&self) -> Arc<Notify> { Arc::clone(&self.safe_mon) }

    /// Subscribes to the event hub to receive mission announcement broadcasts.
    pub(crate) fn subscribe_event_hub(&self) -> broadcast::Receiver<(DateTime<Utc>, String)> {
        self.event_hub.subscribe()
    }

    /// Listens to the `/announcements` Event Source endpoint and broadcasts messages to subscribers.
    ///
    /// Automatically closes on error and logs termination as fatal.
    pub(crate) async fn run_announcement_hub(&self) {
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

    /// Triggers daily full map export and upload at 22:55 UTC.
    ///
    /// This repeats daily and logs errors upon failure.
    ///
    /// # Arguments
    /// * `c_cont` – Shared reference to the `CameraController`.
    pub(crate) async fn run_daily_map_uploader(&self, c_cont: Arc<CameraController>) {
        let now = Utc::now();
        let end_of_day = NaiveTime::from_hms_opt(22, 55, 0).unwrap();
        let upload_t = now.date_naive().and_time(end_of_day);
        let mut next_upload_t = Utc.from_utc_datetime(&upload_t);
        loop {
            let next_upload_dt = (next_upload_t - Utc::now()).to_std().unwrap_or(DT_0_STD);
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

    /// Receive and schedule a secret objective `id` and assigns coordinates to it if valid.
    /// This is called by the user console when assigning a zone to a secret objective.
    ///
    /// Only triggers if the objective is currently active and not expired.
    ///
    /// # Arguments
    /// * `id` – Unique identifier of the objective.
    /// * `zone` – Assigned coordinates `[x_1, y_1, x_2, y_2]`.
    pub(crate) async fn schedule_secret_objective(&self, id: usize, zone: [i32; 4]) {
        let mut secret_obj = self.current_secret_objectives.write().await;
        if let Some(pos) =
            secret_obj.iter().position(|obj| obj.id() == id && obj.end() > Utc::now() && obj.start() < Utc::now() + TimeDelta::hours(4))
        {
            obj!("Received position instructions for secret objective {id} from console!");
            let obj = secret_obj.remove(pos);
            self.zo_mon.send(KnownImgObjective::try_from((obj, zone)).unwrap()).await.unwrap();
        }
    }

    /// Main observation loop that:
    /// - Monitors for safe-mode transitions.
    /// - Periodically polls objectives from the backend.
    /// - Filters and sends active objectives to downstream systems.
    ///
    /// Includes ID caching, secret filtering, and fail-safe alerts.
    #[allow(clippy::cast_precision_loss, clippy::too_many_lines)]
    pub(crate) async fn run_obs_obj_mon(&self) {
        let mut last_objective_check = Utc::now() - Self::OBJ_UPDATE_INTERVAL;
        let mut id_list: HashSet<usize> = HashSet::new();
        Self::prefill_id_list(&mut id_list);
        log!("Starting obs/obj supervisor loop!");
        loop {
            let mut f_cont = self.f_cont_lock.write().await;
            // Update observation and fetch new position
            f_cont.update_observation().await;
            let last_update = Instant::now();

            let is_safe_trans = {
                let current_state = f_cont.state();
                let target_state = f_cont.target_state();
                current_state == FlightState::Transition && target_state.is_none()
            };
            if is_safe_trans {
                warn!("Unplanned Safe Mode Transition Detected! Notifying!");
                self.safe_mon.notify_one();
                self.f_cont_lock.write().await.safe_detected();
            }

            drop(f_cont); // Release the lock early to avoid blocking

            if last_objective_check + Self::OBJ_UPDATE_INTERVAL < Utc::now() {
                let handle = self.f_cont_lock.read().await.client();
                let objective_list = ObjectiveListRequest {}.send_request(&handle).await.unwrap();
                let mut send_img_objs = vec![];
                let mut send_beac_objs = vec![];

                let mut secret_list = self.current_secret_objectives.write().await;
                for img_obj in objective_list.img_objectives() {
                    let obj_on = img_obj.start() < Utc::now() && img_obj.end() > Utc::now();
                    let is_secret = matches!(img_obj.zone_type(), ZoneType::SecretZone(_));
                    let is_future = img_obj.start() > Utc::now();
                    let is_future_short = img_obj.end() < Utc::now() + TimeDelta::hours(5);
                    if !id_list.contains(&img_obj.id()) {
                        if is_secret {
                            secret_list.push(img_obj.clone());
                            id_list.insert(img_obj.id());
                        } else if obj_on || (is_future && is_future_short) {
                            send_img_objs
                                .push(KnownImgObjective::try_from(img_obj.clone()).unwrap());
                        }
                    }
                }
                drop(secret_list);
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

    /// Reads the environment variable `SKIP_OBJ` and adds valid IDs to the internal filter list.
    ///
    /// Used to prevent repeat processing of already completed or irrelevant objectives.
    ///
    /// # Arguments
    /// * `id_list` – A mutable reference to the set of objective IDs.
    fn prefill_id_list(id_list: &mut HashSet<usize>) {
        let done_ids: Vec<Option<usize>> = env::var(Self::ENV_SKIP_OBJ)
            .unwrap_or_default()
            .split(',')
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .map(|s| s.parse::<usize>().ok())
            .collect();
        for done_id in done_ids.into_iter().flatten() {
            info!("Prefilling done obj id list with id: {done_id}");
            id_list.insert(done_id);
        }
    }
}
