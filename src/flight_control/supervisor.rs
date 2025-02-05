use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::flight_state::FlightState;
use fixed::types::I32F32;
use std::sync::Arc;
use tokio::sync::{Notify, RwLock};

pub struct Supervisor {
    f_cont_lock: Arc<RwLock<FlightComputer>>,
    safe_mode_notify: Arc<Notify>,
    reset_pos_monitor: Arc<Notify>,
}

impl Supervisor {
    /// Constant update interval for the `run()` method
    const UPDATE_INTERVAL: std::time::Duration = std::time::Duration::from_millis(500);

    /// Creates a new instance of `Supervisor`
    pub fn new(f_cont_lock: Arc<RwLock<FlightComputer>>) -> Supervisor {
        Self {
            f_cont_lock,
            safe_mode_notify: Arc::new(Notify::new()),
            reset_pos_monitor: Arc::new(Notify::new()),
        }
    }

    pub fn safe_mode_notify(&self) -> Arc<Notify> {
        Arc::clone(&self.safe_mode_notify)
    }

    pub fn reset_pos_monitor(&self) -> Arc<Notify> {
        Arc::clone(&self.reset_pos_monitor)
    }

    pub fn notifiers(&self) -> (Arc<Notify>, Arc<Notify>) {
        (self.reset_pos_monitor(), self.safe_mode_notify())
    }

    /// Starts the supervisor loop to periodically call `update_observation`
    /// and monitor position & state deviations.
    #[allow(clippy::cast_precision_loss)]
    pub async fn run(&self) {
        // TODO: pos monitoring in kalman filter + listening for reset_pos events
        let mut last_pos: Option<Vec2D<I32F32>> = None;
        let mut last_timestamp = chrono::Utc::now();
        let mut last_vel = self.f_cont_lock.read().await.current_vel();

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
                    self.safe_mode_notify.notify_one();
                    self.f_cont_lock.write().await.safe_detected();
                }
            }

            last_pos = Some(current_pos);
            last_timestamp = chrono::Utc::now();
            last_vel = current_vel;

            tokio::time::sleep(Self::UPDATE_INTERVAL).await;
        }
    }
}
