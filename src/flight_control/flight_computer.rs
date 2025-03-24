use super::{
    camera_state::CameraAngle,
    common::{math::MAX_DEC, vec2d::Vec2D},
    flight_state::{FlightState, TRANS_DEL},
};
use crate::flight_control::common::vec2d::WrapDirection;
use crate::flight_control::orbit::ClosedOrbit;
use crate::flight_control::{
    orbit::{BurnSequence, IndexedOrbitPosition},
    task::TaskController,
};
use crate::http_handler::{
    http_client,
    http_request::{
        control_put::ControlSatelliteRequest,
        observation_get::ObservationRequest,
        request_common::{JSONBodyHTTPRequestType, NoBodyHTTPRequestType},
        reset_get::ResetRequest,
    },
};
use crate::{STATIC_ORBIT_VEL, error, fatal, info, log, warn};
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::{I32F32, I64F64};
use num::{ToPrimitive, Zero};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::sync::RwLock;

pub type TurnsClockCClockTup = (
    Vec<(Vec2D<I32F32>, Vec2D<I32F32>)>,
    Vec<(Vec2D<I32F32>, Vec2D<I32F32>)>,
);

/// Represents the core flight computer for satellite control.
/// It manages operations such as state changes, velocity updates,
/// battery charging.
///
/// This system interfaces with an external HTTP-based control system to
/// send requests for managing the satellite’s behavior (e.g., velocity updates,
/// state transitions). Additionally, the file schedules satellite-related
/// activities like image captures and ensures the system stays updated
/// using observations.
///
/// Key methods allow high-level control, including state transitions, camera angle
/// adjustments, and battery-related tasks.
///
/// # Fields
/// - `current_pos`: The current 2D position of the satellite.
/// - `current_vel`: The current 2D velocity of the satellite.
/// - `current_state`: Current operational state of the satellite based on the `FlightState` model.
/// - `current_angle`: The active camera angle for the satellite’s imaging system.
/// - `current_battery`: Indicates the current battery charge.
/// - `max_battery`: Represents the maximum capacity of the satellite battery.
/// - `fuel_left`: Represents the remaining satellite fuel for propulsion/other operations.
/// - `last_observation_timestamp`: Indicates the time of the last system update.
/// - `request_client`: A reference to the HTTP client to send flight control requests.
#[derive(Debug)]
pub struct FlightComputer {
    /// Current position of the satellite in 2D space.
    current_pos: Vec2D<I32F32>,
    /// Current velocity of the satellite in 2D space.
    current_vel: Vec2D<I32F32>,
    /// Current state of the satellite based on `FlightState`.
    current_state: FlightState,
    /// Target State if `current_state` is `FlightState::Transition`
    target_state: Option<FlightState>,
    /// Current angle of the satellite's camera (e.g., Narrow, Normal, Wide).
    current_angle: CameraAngle,
    /// Current battery level of the satellite.
    current_battery: I32F32,
    /// Maximum battery capacity of the satellite.
    max_battery: I32F32,
    /// Remaining fuel level for the satellite operations.
    fuel_left: I32F32,
    /// Timestamp marking the last observation update from the satellite.
    last_observation_timestamp: DateTime<Utc>,
    /// HTTP client for sending requests for satellite operations.
    request_client: Arc<http_client::HTTPClient>,
}

impl FlightComputer {
    /// A constant I32F32 0.0 value for fuel and battery min values
    pub const MIN_0: I32F32 = I32F32::ZERO;
    /// A constant I32F32 100.0 value for fuel and battery max values
    pub const MAX_100: I32F32 = I32F32::lit("100.0");
    /// Constant acceleration in target velocity vector direction
    pub const ACC_CONST: I32F32 = I32F32::lit("0.02");
    /// Constant fuel consumption per accelerating second
    pub const FUEL_CONST: I32F32 = I32F32::lit("0.03");
    /// Maximum decimal places that are used in the observation endpoint for velocity
    pub const VEL_BE_MAX_DECIMAL: u8 = MAX_DEC;
    /// Constant timeout for the `wait_for_condition`-method
    const DEF_COND_TO: u32 = 3000;
    /// Constant timeout for the `wait_for_condition`-method
    const DEF_COND_PI: u16 = 500;
    /// Constant transition to SAFE sleep time for all states
    const TO_SAFE_SLEEP: Duration = Duration::from_secs(60);
    /// Maximum absolute vel change for orbit return
    const MAX_OR_VEL_CHANGE_ABS: I32F32 = I32F32::lit("1.5");
    /// Deviation at which `MAX_VEL_CHANGE_ABS` should occur
    const MAX_OR_VEL_CHANGE_DEV: I32F32 = I32F32::lit("160");
    /// Maximum acceleration time needed for orbit return maneuvers (this is 2*50s, as we
    /// only change velocity by 1.0, and 10s for minor maneuvers)
    const MAX_OR_ACQ_ACC_TIME: I32F32 = I32F32::lit("160");
    /// Maximum time spend in acquisition between burns for orbit returns (this is the distance
    /// travelled during acceleration/brake (2*25) which leaves a maximum of 110 at max speed according to `MAX_OR_VEL_CHANGE_DEV`)
    const MAX_OR_ACQ_TIME: I32F32 = I32F32::lit("156");
    /// Minimum battery used in decision-making for after safe transition
    const AFTER_SAFE_MIN_BATT: I32F32 = I32F32::lit("50");
    const EXIT_SAFE_MIN_BATT: I32F32 = I32F32::lit("10.0");
    /// Constant minimum delay between requests
    pub(crate) const STD_REQUEST_DELAY: Duration = Duration::from_millis(100);
    /// Legal Target States for State Change
    const LEGAL_TARGET_STATES: [FlightState; 3] = [
        FlightState::Acquisition,
        FlightState::Charge,
        FlightState::Comms,
    ];

    pub fn one_time_safe(&mut self) {
        self.current_state = FlightState::Transition;
        self.target_state = None;
    }

    /// Initializes a new `FlightComputer` instance.
    ///
    /// # Arguments
    /// - `request_client`: A reference to the HTTP client for sending simulation requests.
    ///
    /// # Returns
    /// A fully initialized `FlightComputer` with up-to-date field values.
    pub async fn new(request_client: Arc<http_client::HTTPClient>) -> FlightComputer {
        let mut return_controller = FlightComputer {
            current_pos: Vec2D::new(I32F32::zero(), I32F32::zero()),
            current_vel: Vec2D::new(I32F32::zero(), I32F32::zero()),
            current_state: FlightState::Deployment,
            target_state: None,
            current_angle: CameraAngle::Normal,
            current_battery: I32F32::zero(),
            max_battery: I32F32::zero(),
            fuel_left: I32F32::zero(),
            last_observation_timestamp: Utc::now(),
            request_client,
        };
        return_controller.update_observation().await;
        return_controller
    }

    /// Truncates the velocity components to a fixed number of decimal places, as defined by `VEL_BE_MAX_DECIMAL`,
    /// and calculates the remainder (deviation) after truncation.
    ///
    /// # Arguments
    /// - `vel`: A `Vec2D<I32F32>` representing the velocity to be truncated.
    ///
    /// # Returns
    /// A tuple containing:
    /// - A `Vec2D<I32F32>` with truncated velocity components.
    /// - A `Vec2D<I64F64>` representing the fractional deviation from the truncation.
    pub fn trunc_vel(vel: Vec2D<I32F32>) -> (Vec2D<I32F32>, Vec2D<I64F64>) {
        let factor = I32F32::from_num(10f32.powi(i32::from(Self::VEL_BE_MAX_DECIMAL)));
        let factor_f64 = I64F64::from_num(10f64.powi(i32::from(Self::VEL_BE_MAX_DECIMAL)));
        let trunc_x = (vel.x() * factor).floor() / factor;
        let trunc_y = (vel.y() * factor).floor() / factor;
        let dev_x = (I64F64::from_num(vel.x()) * factor_f64).frac() / factor_f64;
        let dev_y = (I64F64::from_num(vel.y()) * factor_f64).frac() / factor_f64;
        (Vec2D::new(trunc_x, trunc_y), Vec2D::new(dev_x, dev_y))
    }

    pub fn round_vel_expand(vel: Vec2D<I32F32>) -> Vec2D<I32F32> {
        let factor = I32F32::from_num(10f32.powi(i32::from(Self::VEL_BE_MAX_DECIMAL)));
        let trunc_x = (vel.x() * factor).round();
        let trunc_y = (vel.y() * factor).round();
        Vec2D::new(trunc_x, trunc_y)
    }
    pub fn round_vel(vel: Vec2D<I32F32>) -> (Vec2D<I32F32>, Vec2D<I64F64>) {
        let factor = I32F32::from_num(10f32.powi(i32::from(Self::VEL_BE_MAX_DECIMAL)));
        let factor_f64 = I64F64::from_num(10f64.powi(i32::from(Self::VEL_BE_MAX_DECIMAL)));
        let trunc_x = (vel.x() * factor).round() / factor;
        let trunc_y = (vel.y() * factor).round() / factor;
        let dev_x = (I64F64::from_num(vel.x()) * factor_f64).frac() / factor_f64;
        let dev_y = (I64F64::from_num(vel.y()) * factor_f64).frac() / factor_f64;
        (Vec2D::new(trunc_x, trunc_y), Vec2D::new(dev_x, dev_y))
    }

    /// Precomputes possible turns of MELVIN, splitting paths into clockwise and counterclockwise
    /// directions based on the initial velocity. These precomputed paths are useful for calculating
    /// optimal burns.
    ///
    /// # Arguments
    /// - `init_vel`: A `Vec2D<I32F32>` representing the initial velocity of the satellite.
    ///
    /// # Returns
    /// A tuple of vectors containing possible turns:
    /// - The first vector contains possible clockwise turns `(position, velocity)`.
    /// - The second vector contains possible counterclockwise turns `(position, velocity)`.
    #[allow(clippy::cast_possible_truncation)]
    pub fn compute_possible_turns(init_vel: Vec2D<I32F32>) -> TurnsClockCClockTup {
        let start_x = init_vel.x();
        let end_x = I32F32::zero();
        let start_y = init_vel.y();
        let end_y = I32F32::zero();

        let step_x =
            if start_x > end_x { -FlightComputer::ACC_CONST } else { FlightComputer::ACC_CONST };
        let step_y =
            if start_y > end_y { -FlightComputer::ACC_CONST } else { FlightComputer::ACC_CONST };

        // Calculates changes along the X-axis while keeping the Y-axis constant.
        let y_const_x_change: Vec<(Vec2D<I32F32>, Vec2D<I32F32>)> = {
            let mut x_pos_vel = Vec::new();
            let step = Vec2D::new(step_x, I32F32::zero());
            let i_last = (start_x / step_x).ceil().abs().to_num::<i32>();
            let mut next_pos = Vec2D::new(I32F32::zero(), I32F32::zero());
            let mut next_vel = init_vel + step;
            x_pos_vel.push((next_pos.round(), next_vel.round_to_2()));
            for i in 0..i_last {
                next_pos = next_pos + next_vel;
                if i == i_last - 1 {
                    next_vel = Vec2D::new(I32F32::zero(), start_y);
                } else {
                    next_vel = next_vel + step;
                }
                x_pos_vel.push((next_pos.round(), next_vel.round_to_2()));
            }
            x_pos_vel
        };

        // Calculates changes along the Y-axis while keeping the X-axis constant.
        let x_const_y_change: Vec<(Vec2D<I32F32>, Vec2D<I32F32>)> = {
            let mut y_pos_vel = Vec::new();
            let step = Vec2D::new(I32F32::zero(), step_y);
            let i_last = (start_y / step_y).ceil().abs().to_num::<i32>();
            let mut next_pos = Vec2D::new(I32F32::zero(), I32F32::zero());
            let mut next_vel = init_vel + step;
            y_pos_vel.push((next_pos.round(), next_vel.round_to_2()));
            for i in 0..i_last {
                next_pos = next_pos + next_vel;
                if i == i_last - 1 {
                    next_vel = Vec2D::new(start_x, I32F32::zero());
                } else {
                    next_vel = next_vel + step;
                }
                y_pos_vel.push((next_pos.round(), next_vel.round_to_2()));
            }
            y_pos_vel
        };

        // Determine the ordering of clockwise and counterclockwise turns based on the direction of steps.
        if step_x.signum() == step_y.signum() {
            (y_const_x_change, x_const_y_change)
        } else {
            (x_const_y_change, y_const_x_change)
        }
    }

    /// Retrieves the current position of the satellite.
    ///
    /// # Returns
    /// A `Vec2D` representing the current satellite position.
    pub fn current_pos(&self) -> Vec2D<I32F32> { self.current_pos }

    /// Retrieves the current position of the satellite.
    ///
    /// # Returns
    /// A `Vec2D` representing the current satellite position.
    pub fn current_angle(&self) -> CameraAngle { self.current_angle }

    /// Retrieves the current position of the velocity.
    ///
    /// # Returns
    /// A `Vec2D` representing the current satellite velocity.
    pub fn current_vel(&self) -> Vec2D<I32F32> { self.current_vel }

    /// Retrieves the maximum battery capacity of the satellite.
    ///
    /// This value fluctuates only due to battery depletion safe mode events.
    ///
    /// # Returns
    /// - A `I32F32` value representing the maximum battery charge.
    pub fn max_battery(&self) -> I32F32 { self.max_battery }

    /// Retrieves the current battery charge level of the satellite.
    ///
    /// # Returns
    /// - A `I32F32` value denoting the battery's current charge level.
    pub fn current_battery(&self) -> I32F32 { self.current_battery }

    /// Retrieves the remaining fuel level of the satellite.
    ///
    /// # Returns
    /// - A `I32F32` value representing the remaining percentage of fuel.
    pub fn fuel_left(&self) -> I32F32 { self.fuel_left }

    /// Retrieves the current operational state of the satellite.
    ///
    /// The state of the satellite determines its behavior, such as charging (`Charge`),
    /// image acquisition (`Acquisition`), ...
    ///
    /// # Returns
    /// - A `FlightState` enum denoting the active operational state.
    pub fn state(&self) -> FlightState { self.current_state }

    /// Retrieves the current target state of the satellite.
    ///
    /// The target state represents the resulting state of a commanded State change.
    ///
    /// # Returns
    /// - A `Option<FlightState>` denoting the target state of the commanded state change.
    pub fn target_state(&self) -> Option<FlightState> { self.target_state }

    /// Retrieves a clone of the HTTP client used by the flight computer for sending requests.
    ///
    /// # Returns
    /// - An `Arc<http_client::HTTPClient>` which represents the HTTP client instance.
    pub fn client(&self) -> Arc<http_client::HTTPClient> { Arc::clone(&self.request_client) }

    /// Sends a reset request to the satellite's HTTP control system.
    ///
    /// This function invokes an HTTP request to reset MELVIN, and it expects the request to succeed.
    ///
    /// # Panics
    /// - If the reset request fails, this method will panic with an error message.
    pub async fn reset(&self) {
        ResetRequest {}.send_request(&self.request_client).await.expect("ERROR: Failed to reset");
        Self::wait_for_duration(Duration::from_secs(4)).await;
        log!("Reset request complete.");
    }

    /// Indicates that a `Supervisor` detected a safe mode event
    pub fn safe_detected(&mut self) { self.target_state = Some(FlightState::Safe); }

    /// Waits for a given amount of time with debug prints, this is a static method.
    ///
    /// # Arguments
    /// - `sleep`: The duration for which the system should wait.
    pub async fn wait_for_duration(sleep: Duration) {
        if sleep.as_secs() == 0 {
            log!("Wait call rejected! Duration was 0!");
            return;
        }
        info!("Waiting for {} seconds!", sleep.as_secs());
        tokio::time::sleep(sleep).await;
    }

    /// Waits for a condition to be met within a specified timeout.
    ///
    /// # Arguments
    /// - `self_lock`: A `RwLock<Self>` reference to the active flight computer.
    /// - `condition`: A closure that takes a reference to `Self` and returns a `bool`.
    /// - `rationale`: A string describing the purpose of the wait.
    /// - `timeout_millis`: Maximum time in milliseconds to wait for the condition to be met.
    /// - `poll_interval`: Interval in milliseconds to check the condition.
    ///
    /// # Behavior
    /// - The method continuously polls the condition at the specified interval until:
    ///   - The condition returns `true`, or
    ///   - The timeout expires.
    /// - Logs the rationale and results of the wait.
    async fn wait_for_condition<F>(
        self_lock: &RwLock<Self>,
        (condition, rationale): (F, String),
        timeout_millis: u32,
        poll_interval: u16,
        mute: bool,
    ) where
        F: Fn(&Self) -> bool,
    {
        if !mute {
            log!("Waiting for condition: {rationale}");
        }
        let start_time = Instant::now();
        while start_time.elapsed().as_millis() < u128::from(timeout_millis) {
            let cond = { condition(&*self_lock.read().await) };
            if cond {
                if !mute {
                    let dt = start_time.elapsed().as_millis();
                    log!("Condition met after {dt} ms");
                }
                return;
            }
            tokio::time::sleep(Duration::from_millis(u64::from(poll_interval))).await;
        }
        if mute {
            warn!("Condition: {rationale} not met after {timeout_millis:#?} ms");
        } else {
            warn!("Condition not met after {timeout_millis:#?} ms");
        }
    }

    pub async fn escape_safe(self_lock: Arc<RwLock<Self>>, force_charge: bool) {
        let target_state = {
            let init_batt = self_lock.read().await.current_battery();
            if init_batt <= Self::AFTER_SAFE_MIN_BATT || force_charge {
                FlightState::Charge
            } else {
                FlightState::Acquisition
            }
        };
        info!("Safe Mode Runtime initiated. Transitioning back to {target_state} asap.");
        Self::wait_for_duration(Self::TO_SAFE_SLEEP).await;
        let cond_not_trans = (
            // TODO: later this condition must be == FlightState::Safe
            |cont: &FlightComputer| cont.state() != FlightState::Transition,
            format!("State is not {}", FlightState::Transition),
        );
        Self::wait_for_condition(
            &self_lock,
            cond_not_trans,
            Self::DEF_COND_TO,
            Self::DEF_COND_PI,
            false,
        )
        .await;
        let state = self_lock.read().await.state();
        if state != FlightState::Safe {
            error!("State is not safe but {}", state);
        }
        let cond_min_charge = (
            |cont: &FlightComputer| cont.current_battery() > Self::EXIT_SAFE_MIN_BATT,
            format!("Battery level is higher than {}", Self::EXIT_SAFE_MIN_BATT),
        );
        Self::wait_for_condition(
            &self_lock,
            cond_min_charge,
            450_000,
            Self::DEF_COND_PI,
            false,
        )
        .await;
        Self::set_state_wait(self_lock, target_state).await;
    }

    #[allow(clippy::cast_possible_wrap)]
    pub async fn get_to_comms(self_lock: Arc<RwLock<Self>>) -> DateTime<Utc> {
        if self_lock.read().await.state() == FlightState::Comms {
            let batt_diff =
                self_lock.read().await.current_battery() - TaskController::MIN_BATTERY_THRESHOLD;
            let rem_t = (batt_diff / FlightState::Comms.get_charge_rate()).abs().ceil();
            let add_t = TimeDelta::seconds(rem_t.to_num::<i64>()).min(TimeDelta::seconds(
                TaskController::IN_COMMS_SCHED_SECS as i64,
            ));
            return Utc::now() + add_t;
        }
        let charge_dt = Self::get_charge_dt_comms(&self_lock).await;
        log!("Charge time for comms: {}", charge_dt);

        if charge_dt > 0 {
            FlightComputer::set_state_wait(Arc::clone(&self_lock), FlightState::Charge).await;
            FlightComputer::wait_for_duration(Duration::from_secs(charge_dt)).await;
            FlightComputer::set_state_wait(self_lock, FlightState::Comms).await;
        } else {
            FlightComputer::set_state_wait(self_lock, FlightState::Comms).await;
        }
        Utc::now() + TimeDelta::seconds(TaskController::IN_COMMS_SCHED_SECS as i64)
    }

    #[allow(clippy::cast_possible_wrap)]
    pub async fn escape_if_comms(self_lock: Arc<RwLock<Self>>) -> DateTime<Utc> {
        let (state, batt) = {
            let f_cont = self_lock.read().await;
            (f_cont.state(), f_cont.current_battery())
        };
        if state == FlightState::Comms {
            let half_batt =
                (TaskController::MAX_BATTERY_THRESHOLD + TaskController::MIN_BATTERY_THRESHOLD) / 2;
            if batt > half_batt {
                FlightComputer::set_state_wait(Arc::clone(&self_lock), FlightState::Acquisition)
                    .await;
            } else {
                FlightComputer::set_state_wait(Arc::clone(&self_lock), FlightState::Charge).await;
            }
        }
        Utc::now()
    }

    #[allow(clippy::cast_possible_wrap)]
    pub async fn get_to_comms_dt_est(self_lock: Arc<RwLock<Self>>) -> DateTime<Utc> {
        let t_time = FlightState::Charge.td_dt_to(FlightState::Comms);
        if self_lock.read().await.state() == FlightState::Comms {
            let batt_diff =
                self_lock.read().await.current_battery() - TaskController::MIN_BATTERY_THRESHOLD;
            let rem_t = (batt_diff / FlightState::Comms.get_charge_rate().abs()).abs().ceil();
            return Utc::now() + TimeDelta::seconds(rem_t.to_num::<i64>());
        }
        let charge_dt = Self::get_charge_dt_comms(&self_lock).await;

        if charge_dt > 0 {
            Utc::now() + TimeDelta::seconds(charge_dt as i64) + t_time * 2
        } else {
            Utc::now() + t_time
        }
    }

    pub async fn get_to_static_orbit_vel(self_lock: &Arc<RwLock<Self>>) {
        let orbit_vel = Vec2D::from(STATIC_ORBIT_VEL);
        let (batt, vel) = {
            let f_cont = self_lock.read().await;
            (f_cont.current_battery(), f_cont.current_vel())
        };
        let vel_change_dt =
            Duration::from_secs_f32((orbit_vel.euclid_distance(&vel) / Self::ACC_CONST).to_num::<f32>());
        let charge_needed = {
            let acq_acc_db = FlightState::Acquisition.get_charge_rate() + FlightState::ACQ_ACC_ADDITION;
            let or_vel_corr_db = I32F32::from_num(vel_change_dt.as_secs()) * acq_acc_db;
            let or_db = Self::max_or_maneuver_charge();
            TaskController::MIN_BATTERY_THRESHOLD - 1 * (or_vel_corr_db + or_db)
        };
        log!("Getting back to orbit velocity {orbit_vel}. Minimum charge needed: {charge_needed}");
        if batt < charge_needed {
            FlightComputer::charge_full_wait(self_lock).await;
        }
        let state = self_lock.read().await.state();
        if !matches!(state, FlightState::Acquisition) {
            FlightComputer::set_state_wait(Arc::clone(self_lock), FlightState::Acquisition).await;
        }
        FlightComputer::set_vel_wait(Arc::clone(self_lock), orbit_vel, true).await;
    }

    async fn get_charge_dt_comms(self_lock: &Arc<RwLock<Self>>) -> u64 {
        let batt_diff = (self_lock.read().await.current_battery()
            - TaskController::MIN_COMMS_START_CHARGE)
            .min(I32F32::zero());
        (-batt_diff / FlightState::Charge.get_charge_rate()).ceil().to_num::<u64>()
    }

    pub async fn charge_full_wait(self_lock: &Arc<RwLock<Self>>) {
        let max_batt = self_lock.read().await.max_battery;
        Self::charge_to_wait(self_lock, max_batt).await;
    }

    pub async fn charge_to_wait(self_lock: &Arc<RwLock<Self>>, target_batt: I32F32) {
        let state = self_lock.read().await.state();
        if state == FlightState::Safe {
            FlightComputer::escape_safe(Arc::clone(self_lock), true).await;
            FlightComputer::set_state_wait(Arc::clone(self_lock), FlightState::Charge).await;
        } else {
            FlightComputer::set_state_wait(Arc::clone(self_lock), FlightState::Charge).await;
        }
        let batt = self_lock.read().await.current_battery();
        let dt = (target_batt - batt) / FlightState::Charge.get_charge_rate();
        Self::wait_for_duration(Duration::from_secs(dt.to_num::<u64>())).await;
    }

    /// Transitions the satellite to a new operational state and waits for transition completion.
    ///
    /// # Arguments
    /// - `self_lock`: A `RwLock<Self>` reference to the active flight computer.
    /// - `new_state`: The target operational state.
    pub async fn set_state_wait(self_lock: Arc<RwLock<Self>>, new_state: FlightState) {
        let init_state = { self_lock.read().await.current_state };
        if new_state == init_state {
            log!("State already set to {new_state}");
            return;
        } else if !Self::LEGAL_TARGET_STATES.contains(&new_state) {
            fatal!("State {new_state} is not a legal target state");
        } else if init_state == FlightState::Transition {
            fatal!(" State cant be changed when in {init_state}");
        }
        self_lock.write().await.target_state = Some(new_state);
        self_lock.read().await.set_state(new_state).await;

        let transition_t = TRANS_DEL.get(&(init_state, new_state)).unwrap_or_else(|| {
            fatal!("({init_state}, {new_state}) not in TRANSITION_DELAY_LOOKUP")
        });

        Self::wait_for_duration(*transition_t).await;
        let cond = (
            |cont: &FlightComputer| cont.state() == new_state,
            format!("State equals {new_state}"),
        );
        Self::wait_for_condition(
            &self_lock,
            cond,
            Self::DEF_COND_TO,
            Self::DEF_COND_PI,
            false,
        )
        .await;
        self_lock.write().await.target_state = None;
    }

    /// Adjusts the velocity of the satellite and waits until the target velocity is reached.
    ///
    /// # Arguments
    /// - `self_lock`: A `RwLock<Self>` reference to the active flight computer.
    /// - `new_vel`: The target velocity vector.
    pub async fn set_vel_wait(self_lock: Arc<RwLock<Self>>, new_vel: Vec2D<I32F32>, mute: bool) {
        let (current_state, current_vel) = {
            let f_cont_read = self_lock.read().await;
            (f_cont_read.state(), f_cont_read.current_vel())
        };
        if current_state != FlightState::Acquisition {
            fatal!("Velocity cant be changed in state {current_state}");
        }
        let vel_change_dt = Duration::from_secs_f32(
            (new_vel.euclid_distance(&current_vel) / Self::ACC_CONST).to_num::<f32>(),
        );
        self_lock.read().await.set_vel(new_vel, mute).await;
        if vel_change_dt.as_secs() > 0 {
            Self::wait_for_duration(vel_change_dt).await;
        }
        let comp_new_vel = Self::round_vel_expand(new_vel);
        let cond = (
            |cont: &FlightComputer| Self::round_vel_expand(cont.current_vel()) == comp_new_vel,
            format!("Scaled Vel equals {new_vel}"),
        );
        Self::wait_for_condition(&self_lock, cond, Self::DEF_COND_TO, Self::DEF_COND_PI, mute)
            .await;
    }

    /// Adjusts the satellite's camera angle and waits until the target angle is reached.
    ///
    /// # Arguments
    /// - `self_lock`: A `RwLock<Self>` reference to the active flight computer.
    /// - `new_angle`: The target camera angle.
    ///
    /// # Behavior
    /// - If the current angle matches the new angle, logs the status and exits.
    /// - Checks if the current state permits changing the camera angle.
    ///   If not, it panics with a fatal error.
    /// - Sets the new angle and waits until the system confirms it has been applied.
    pub async fn set_angle_wait(self_lock: Arc<RwLock<Self>>, new_angle: CameraAngle) {
        let (current_angle, current_state) = {
            let f_cont_read = self_lock.read().await;
            (f_cont_read.current_angle, f_cont_read.state())
        };
        if current_angle == new_angle {
            log!("Angle already set to {new_angle}");
            return;
        }
        if current_state != FlightState::Acquisition {
            fatal!("Angle cant be changed in state {current_state}");
        }

        self_lock.read().await.set_angle(new_angle).await;
        let cond = (
            |cont: &FlightComputer| cont.current_angle() == new_angle,
            format!("Lens equals {new_angle}"),
        );
        Self::wait_for_condition(
            &self_lock,
            cond,
            Self::DEF_COND_TO,
            Self::DEF_COND_PI,
            false,
        )
        .await;
    }

    /// Executes a sequence of thruster burns that affect the trajectory of MELVIN.
    ///
    /// # Arguments
    /// - `self_lock`: A `RwLock<Self>` reference to the active flight computer.
    /// - `burn_sequence`: A reference to the sequence of executed thruster burns.
    pub async fn execute_burn(self_lock: Arc<RwLock<Self>>, burn: &BurnSequence) {
        let burn_start = Utc::now();
        for vel_change in burn.sequence_vel() {
            let st = tokio::time::Instant::now();
            let dt = Duration::from_secs(1);
            FlightComputer::set_vel_wait(Arc::clone(&self_lock), *vel_change, true).await;
            let el = st.elapsed();
            if el < dt {
                tokio::time::sleep(dt).await;
            }
        }
        let target_pos = burn.sequence_pos().last().unwrap();
        let target_vel = burn.sequence_vel().last().unwrap();
        let (pos, vel) = {
            let f_cont = self_lock.read().await;
            (f_cont.current_pos(), f_cont.current_vel())
        };
        let burn_dt = (Utc::now() - burn_start).num_seconds();
        log!(
            "Burn sequence finished after {burn_dt}s! Position: {pos}, Velocity: {vel}, expected Position: {target_pos}, expected Velocity: {target_vel}."
        );
    }

    pub async fn or_maneuver(self_lock: Arc<RwLock<Self>>, c_o: Arc<RwLock<ClosedOrbit>>) -> usize {
        let o_unlocked = c_o.read().await;
        let (mut pos, vel) = {
            let f_cont = self_lock.read().await;
            (f_cont.current_pos(), f_cont.current_vel())
        };
        while o_unlocked.will_visit(pos) {
            let (ax, dev) = o_unlocked.get_closest_deviation(pos);
            let (dv, h_dt) = Self::compute_vmax_and_hold_time(dev);
            log!("Computed Orbit Return. Deviation on {ax} is {dev} and vel is {vel}.");
            let corr_v = vel + Vec2D::from_axis_and_val(ax, dv);
            log!("Correction velocity is {corr_v}, ramping by {dv:.2}. Hold time will be {h_dt}s.");
            FlightComputer::set_vel_wait(Arc::clone(&self_lock), corr_v, false).await;
            if h_dt > 0 {
                FlightComputer::wait_for_duration(Duration::from_secs(h_dt)).await;
            }
            FlightComputer::set_vel_wait(Arc::clone(&self_lock), vel, false).await;
            pos = self_lock.read().await.current_pos();
        }
        o_unlocked.get_i(pos).unwrap()
    }

    pub fn max_or_maneuver_charge() -> I32F32 {
        let acq_db = FlightState::Acquisition.get_charge_rate();
        let acq_acc_db = acq_db + FlightState::ACQ_ACC_ADDITION;
        Self::MAX_OR_ACQ_ACC_TIME * acq_acc_db + Self::MAX_OR_ACQ_TIME * acq_db
    }

    fn compute_vmax_and_hold_time(dev: I32F32) -> (I32F32, u64) {
        // Try triangular profile first (no cruising)
        let dv_triang = dev.signum() * (Self::ACC_CONST * dev.abs()).sqrt();
        if dv_triang.abs() <= Self::MAX_OR_VEL_CHANGE_ABS {
            // Just accelerate to vmax_triangular and decelerate
            (dv_triang, 0)
        } else {
            // Trapezoidal profile: accelerate to vmax_limit, hold, then decelerate
            let t_ramp = Self::MAX_OR_VEL_CHANGE_ABS / Self::ACC_CONST;
            let d_ramp = I32F32::from_num(0.5) * Self::MAX_OR_VEL_CHANGE_ABS * t_ramp; // distance per ramp
            let d_hold = dev.abs() - 2 * d_ramp;
            let t_hold = (d_hold / Self::MAX_OR_VEL_CHANGE_ABS).floor().to_num::<u64>();
            (dev.signum() * Self::MAX_OR_VEL_CHANGE_ABS, t_hold)
        }
    }

    pub async fn detumble_to(
        self_lock: Arc<RwLock<Self>>,
        mut target: Vec2D<I32F32>,
        lens: CameraAngle,
    ) -> DateTime<Utc> {
        let ticker: i32 = 0;
        let max_speed = lens.get_max_speed();
        let detumble_start = Utc::now();

        let start_pos = self_lock.read().await.current_pos();
        let mut to_target = start_pos.to(&target);
        let mut dt;
        let mut dx;
        let mut last_to_target = to_target;

        loop {
            let (pos, vel) = {
                let f_locked = self_lock.read().await;
                (f_locked.current_pos(), f_locked.current_vel())
            };
            to_target = pos.to(&target);

            // sudden jumps mean we wrapped and we need to wrap the corresponding coordinate too
            if to_target.x().abs() > 2 * last_to_target.x().abs() {
                target = target.wrap_by(&WrapDirection::WrapX);
                to_target = pos.to(&target);
            }
            if to_target.y().abs() > 2 * last_to_target.y().abs() {
                target = target.wrap_by(&WrapDirection::WrapY);
                to_target = pos.to(&target);
            }
            last_to_target = to_target;
            dt = to_target.abs() / vel.abs();
            dx = to_target - (vel.normalize() * dt);

            let acc = dx.normalize() * Self::ACC_CONST;
            let overspeed = vel.abs() > max_speed;
            if ticker % 10 == 0 {
                log!("Detumbling: DX: {dx}, direct DT: {dt:2}s");
                if overspeed {
                    let overspeed_amount = vel.abs() - max_speed;
                    warn!("Overspeeding by {overspeed_amount}");
                }
            }
            let cond = if overspeed {
                let min_brake_dt = ((vel.abs() - max_speed) / Self::ACC_CONST).to_num::<i64>();
                overspeed && (dt.to_num::<i64>() < 10 + min_brake_dt)
            } else {
                dt.to_num::<i64>() < 10
            };
            if dx.abs() < I32F32::lit("1.0") || cond {
                let detumble_dt = (Utc::now() - detumble_start).num_seconds();
                log!(
                    "Detumbling finished after {detumble_dt}s with remaining DX: {dx} and dt {dt:2}s"
                );
                if overspeed {
                    todo!();
                } else {
                    self_lock.write().await.set_vel(vel, true).await;
                }
                FlightComputer::set_angle_wait(Arc::clone(&self_lock), lens).await;
                return Utc::now() + TimeDelta::seconds(dt.to_num::<i64>());
            }
            let (new_vel, _) = FlightComputer::trunc_vel(vel + acc);
            self_lock.write().await.set_vel(new_vel, true).await;
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }

    /// Updates the satellite's internal fields with the latest observation data.
    pub async fn update_observation(&mut self) {
        loop {
            if let Ok(obs) = (ObservationRequest {}.send_request(&self.request_client).await) {
                self.current_pos =
                    Vec2D::from((I32F32::from_num(obs.pos_x()), I32F32::from_num(obs.pos_y())));
                self.current_vel =
                    Vec2D::from((I32F32::from_num(obs.vel_x()), I32F32::from_num(obs.vel_y())));
                self.current_state = FlightState::from(obs.state());
                self.current_angle = CameraAngle::from(obs.angle());
                self.last_observation_timestamp = obs.timestamp();
                self.current_battery =
                    I32F32::from_num(obs.battery()).clamp(Self::MIN_0, Self::MAX_100);
                self.max_battery =
                    I32F32::from_num(obs.max_battery()).clamp(Self::MIN_0, Self::MAX_100);
                self.fuel_left = I32F32::from_num(obs.fuel()).clamp(Self::MIN_0, Self::MAX_100);
                return;
            }
            error!("Unnoticed HTTP Error in updateObservation()");
            tokio::time::sleep(Self::STD_REQUEST_DELAY).await;
        }
    }

    /// Sets the satellite’s `FlightState`.
    ///
    /// # Arguments
    /// - `new_state`: The new operational state.
    async fn set_state(&self, new_state: FlightState) {
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel.x().to_f64().unwrap(),
            vel_y: self.current_vel.y().to_f64().unwrap(),
            camera_angle: self.current_angle.into(),
            state: new_state.into(),
        };
        loop {
            if req.send_request(&self.request_client).await.is_ok() {
                info!("State change started to {new_state}");
                return;
            }
            error!("Unnoticed HTTP Error in updateObservation()");
            tokio::time::sleep(Self::STD_REQUEST_DELAY).await;
        }
    }

    /// Sets the satellite’s velocity. The input velocity should only have two decimal places after comma.
    ///
    /// # Arguments
    /// - `new_vel`: The new velocity.
    async fn set_vel(&self, new_vel: Vec2D<I32F32>, mute: bool) {
        let (vel, _) = Self::round_vel(new_vel);
        let req = ControlSatelliteRequest {
            vel_x: vel.x().to_f64().unwrap(),
            vel_y: vel.y().to_f64().unwrap(),
            camera_angle: self.current_angle.into(),
            state: self.current_state.into(),
        };
        loop {
            if req.send_request(&self.request_client).await.is_ok() {
                if !mute {
                    info!("Velocity change commanded to [{}, {}]", vel.x(), vel.y());
                }
                return;
            }
            error!("Unnoticed HTTP Error in set_vel()");
            tokio::time::sleep(Self::STD_REQUEST_DELAY).await;
        }
    }

    /// Sets the satellite’s `CameraAngle`
    ///
    /// # Arguments
    /// - `new_angle`: The new Camera Angle.
    async fn set_angle(&self, new_angle: CameraAngle) {
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel.x().to_f64().unwrap(),
            vel_y: self.current_vel.y().to_f64().unwrap(),
            camera_angle: new_angle.into(),
            state: self.current_state.into(),
        };
        loop {
            if req.send_request(&self.request_client).await.is_ok() {
                info!("Angle change commanded to {new_angle}");
                return;
            }
            error!("Unnoticed HTTP Error in setAngle()");
            tokio::time::sleep(Self::STD_REQUEST_DELAY).await;
        }
    }

    /// Predicts the satellite’s position after a specified time interval.
    ///
    /// # Arguments
    /// - `time_delta`: The time interval for prediction.
    ///
    /// # Returns
    /// - A `Vec2D<I32F32>` representing the satellite’s predicted position.
    pub fn pos_in_dt(&self, now: IndexedOrbitPosition, dt: TimeDelta) -> IndexedOrbitPosition {
        let pos = self.current_pos
            + (self.current_vel * I32F32::from_num(dt.num_seconds())).wrap_around_map();
        let t = Utc::now() + dt;
        now.new_from_future_pos(pos, t)
    }

    pub fn batt_in_dt(&self, dt: TimeDelta) -> I32F32 {
        self.current_battery
            + (self.current_state.get_charge_rate() * I32F32::from_num(dt.num_seconds()))
    }
}
