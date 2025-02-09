use std::{collections::HashSet, sync::Arc};
use crate::{
    flight_control::{
        common::vec2d::Vec2D,
        flight_computer::FlightComputer,
        flight_state::FlightState,
        objective::{
            objective_type::ObjectiveType,
            known_img_objective::KnownImgObjective,
            secret_img_objective::SecretImgObjective,
        },
    },
    http_handler::http_request::{objective_list_get::ObjectiveListRequest, request_common::NoBodyHTTPRequestType},
};
use fixed::types::I32F32;
use tokio::sync::{mpsc, mpsc::Receiver, Notify, RwLock};

pub struct Supervisor {
    f_cont_lock: Arc<RwLock<FlightComputer>>,
    safe_mode_monitor: Arc<Notify>,
    reset_pos_monitor: Arc<Notify>,
    objective_monitor: mpsc::Sender<ObjectiveType>,
}

impl Supervisor {
    /// Constant update interval for observation updates in the `run()` method
    const OBS_UPDATE_INTERVAL: std::time::Duration = std::time::Duration::from_millis(500);
    /// Constant update interval for objective updates in the `run()` method
    const OBJ_UPDATE_INTERVAL: chrono::TimeDelta = chrono::TimeDelta::seconds(300);
    /// Creates a new instance of `Supervisor`
    pub fn new(f_cont_lock: Arc<RwLock<FlightComputer>>) -> (Supervisor, Receiver<ObjectiveType>) {
        let (tx, rx) = mpsc::channel(10);
        (
            Self {
                f_cont_lock,
                safe_mode_monitor: Arc::new(Notify::new()),
                reset_pos_monitor: Arc::new(Notify::new()),
                objective_monitor: tx,
            },
            rx,
        )
    }

    pub fn safe_mode_monitor(&self) -> Arc<Notify> { Arc::clone(&self.safe_mode_monitor) }

    pub fn reset_pos_monitor(&self) -> Arc<Notify> { Arc::clone(&self.reset_pos_monitor) }

    /// Starts the supervisor loop to periodically call `update_observation`
    /// and monitor position & state deviations.
    #[allow(clippy::cast_precision_loss)]
    pub async fn run(&self) {
        // TODO: pos monitoring in kalman filter + listening for reset_pos events
        let mut last_pos: Option<Vec2D<I32F32>> = None;
        let mut last_timestamp = chrono::Utc::now();
        let mut last_vel = self.f_cont_lock.read().await.current_vel();
        let mut last_objective_check = chrono::Utc::now() - Self::OBJ_UPDATE_INTERVAL;
        let mut obj_id_list: HashSet<usize> = HashSet::new();
        loop {
            let mut f_cont = self.f_cont_lock.write().await;
            // Update observation and fetch new position
            f_cont.update_observation().await;

            let current_pos = f_cont.current_pos();
            let current_vel = f_cont.current_vel();
            let is_safe_trans = {
                let current_state = f_cont.state();
                let target_state = f_cont.target_state();
                current_state == FlightState::Transition && target_state.is_none()
            };

            drop(f_cont); // Release the lock early to avoid blocking

            if let Some(previous_pos) = last_pos {
                let dt = chrono::Utc::now() - last_timestamp;
                let dt_secs = dt.num_milliseconds() as f32 / 1000.0;

                let expected_pos = previous_pos + last_vel * I32F32::from_num(dt_secs);

                let expected_pos_wrapped = expected_pos.wrap_around_map();
                // TODO: this diff value should also consider wrapping (if actual pos wrapped, but expected didnt)
                let diff = current_pos - expected_pos_wrapped;
                /*
                println!(
                    "[INFO] Position tracking: Current: {current_pos}, \
                    Expected: {expected_pos_wrapped}, Diff: {diff}");
                    */
                // Check flight state and handle safe mode (placeholder for now)
                if is_safe_trans {
                    println!("[WARN] Unplanned Safe Mode Transition Detected! Notifying!");
                    self.safe_mode_monitor.notify_one();
                    self.f_cont_lock.write().await.safe_detected();
                }
            }

            last_pos = Some(current_pos);
            last_timestamp = chrono::Utc::now();
            last_vel = current_vel;

            if last_objective_check + Self::OBJ_UPDATE_INTERVAL < chrono::Utc::now() {
                let handle = self.f_cont_lock.read().await.client();
                let objective_list = ObjectiveListRequest {}.send_request(&handle).await.unwrap();
                for img_obj in objective_list.img_objectives() {
                    let obj_on = img_obj.start() < chrono::Utc::now() && img_obj.end() > chrono::Utc::now();
                    let obj_known = obj_id_list.contains(&img_obj.id());
                    if obj_on && !obj_known {
                        let monitor_send = {
                            if img_obj.is_secret() {
                                ObjectiveType::KnownImgObj(KnownImgObjective::try_from(img_obj.clone()).unwrap())
                            } else {
                                ObjectiveType::SecretImgObj(SecretImgObjective::from(img_obj.clone()))
                            }
                        };
                        obj_id_list.insert(img_obj.id());
                        self.objective_monitor.send(monitor_send).await.unwrap();
                    }
                }
                last_objective_check = chrono::Utc::now();
            }

            tokio::time::sleep(Self::OBS_UPDATE_INTERVAL).await;
        }
    }
}
