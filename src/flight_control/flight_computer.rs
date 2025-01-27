use super::{
    camera_state::CameraAngle,
    common::vec2d::Vec2D,
    flight_state::{FlightState, TRANSITION_DELAY_LOOKUP},
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
use std::{sync::Arc, time::Duration};
use std::ops::Index;
use tokio::sync::RwLock;
use crate::flight_control::orbit::index::IndexedOrbitPosition;

type TurnsClockCClockTup = (Vec<(Vec2D<f32>, Vec2D<f32>)>, Vec<(Vec2D<f32>, Vec2D<f32>)>);

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
    current_pos: Vec2D<f32>,
    /// Current velocity of the satellite in 2D space.
    current_vel: Vec2D<f32>,
    /// Current state of the satellite based on `FlightState`.
    current_state: FlightState,
    /// Current angle of the satellite's camera (e.g., Narrow, Normal, Wide).
    current_angle: CameraAngle,
    /// Current battery level of the satellite.
    current_battery: f32,
    /// Maximum battery capacity of the satellite.
    max_battery: f32, // this is an artifact caused by dumb_main
    /// Remaining fuel level for the satellite operations.
    fuel_left: f32,
    /// Timestamp marking the last observation update from the satellite.
    last_observation_timestamp: chrono::DateTime<chrono::Utc>,
    /// HTTP client for sending requests for satellite operations.
    request_client: Arc<http_client::HTTPClient>,
}

pub enum ChargeCommand {
    TargetCharge(f32),
    Duration(chrono::TimeDelta),
}

impl FlightComputer {
    /// Constant acceleration in target velocity vector direction
    pub const ACC_CONST: f32 = 0.02;
    /// Constant fuel consumption per accelerating second
    pub const FUEL_CONST: f32 = 0.03;
    /// Maximum decimal places that are used in the observation endpoint for velocity
    const VEL_BE_MAX_DECIMAL: u8 = 2;
    /// Constant timeout for the `wait_for_condition`-method
    const DEF_COND_TO: u16 = 3000;
    /// Constant timeout for the `wait_for_condition`-method
    const DEF_COND_PI: u16 = 500;
    /// Legal Target States for State Change
    const LEGAL_TARGET_STATES: [FlightState; 3] = [
        FlightState::Acquisition,
        FlightState::Charge,
        FlightState::Comms,
    ];

    /// Initializes a new `FlightComputer` instance.
    ///
    /// # Arguments
    /// - `request_client`: A reference to the HTTP client for sending simulation requests.
    ///
    /// # Returns
    /// A fully initialized `FlightComputer` with up-to-date field values.
    pub async fn new(request_client: Arc<http_client::HTTPClient>) -> FlightComputer {
        let mut return_controller = FlightComputer {
            current_pos: Vec2D::new(0.0, 0.0),
            current_vel: Vec2D::new(0.0, 0.0),
            current_state: FlightState::Safe,
            current_angle: CameraAngle::Normal,
            current_battery: 0.0,
            max_battery: 0.0,
            fuel_left: 0.0,
            last_observation_timestamp: chrono::Utc::now(),
            request_client,
        };
        return_controller.update_observation().await;
        return_controller
    }
    
    pub fn trunc_vel(vel: Vec2D<f32>) -> Vec2D<f32> {
        let factor = 10f32.powi(i32::from(Self::VEL_BE_MAX_DECIMAL));
        Vec2D::new(
            (vel.x() * factor).floor() / factor, 
            (vel.y() * factor).floor() / factor
        )
    }
    
    pub fn compute_possible_turns(init_vel: Vec2D<f32>) -> TurnsClockCClockTup {
        let init_vel_clock_unit = init_vel.perp_unit(true);
        let init_vel_c_clock_unit = init_vel.perp_unit(false);

        let mut acc_clock = init_vel_clock_unit * FlightComputer::ACC_CONST;
        let mut acc_c_clock = init_vel_c_clock_unit * FlightComputer::ACC_CONST;
        let mut vel_clock = FlightComputer::trunc_vel(init_vel + acc_clock);
        let mut vel_c_clock = FlightComputer::trunc_vel(init_vel + acc_c_clock);


        let mut pos_clock: Vec2D<f32> = Vec2D::zero();
        let mut pos_c_clock: Vec2D<f32> = Vec2D::zero();

        let mut pos_vel_clock = vec![(pos_clock, vel_clock)];
        let mut pos_vel_c_clock = vec![(pos_c_clock, vel_c_clock)];

        while !vel_clock.is_clockwise_to(&init_vel_c_clock_unit).unwrap_or(false){
            (pos_clock, vel_clock) = *pos_vel_clock.last().unwrap();
            pos_clock = pos_clock + vel_clock;

            acc_clock = vel_clock.perp_unit(true) * FlightComputer::ACC_CONST;
            vel_clock = FlightComputer::trunc_vel(vel_clock + acc_clock);

            pos_vel_clock.push((pos_clock, vel_clock));
        }

        while vel_c_clock.is_clockwise_to(&init_vel_c_clock_unit).unwrap_or(false){
            (pos_c_clock, vel_c_clock) = *pos_vel_c_clock.last().unwrap();
            pos_c_clock = pos_c_clock + vel_c_clock;

            acc_c_clock = vel_c_clock.perp_unit(true) * FlightComputer::ACC_CONST;
            vel_c_clock = FlightComputer::trunc_vel(vel_c_clock + acc_c_clock);

            pos_vel_c_clock.push((pos_c_clock, vel_c_clock));
        }

        (pos_vel_clock, pos_vel_c_clock)
    }

    /// Retrieves the current position of the satellite.
    ///
    /// # Returns
    /// A `Vec2D` representing the current satellite position.
    pub fn current_pos(&self) -> Vec2D<f32> { self.current_pos }

    /// Retrieves the current position of the satellite.
    ///
    /// # Returns
    /// A `Vec2D` representing the current satellite position.
    pub fn current_angle(&self) -> CameraAngle { self.current_angle }

    /// Retrieves the current position of the velocity.
    ///
    /// # Returns
    /// A `Vec2D` representing the current satellite velocity.
    pub fn current_vel(&self) -> Vec2D<f32> { self.current_vel }

    /// Retrieves the maximum battery capacity of the satellite.
    ///
    /// This value fluctuates only due to battery depletion safe mode events.
    ///
    /// # Returns
    /// - A `f32` value representing the maximum battery charge.
    pub fn max_battery(&self) -> f32 { self.max_battery }

    /// Retrieves the current battery charge level of the satellite.
    ///
    /// # Returns
    /// - A `f32` value denoting the battery's current charge level.
    pub fn current_battery(&self) -> f32 { self.current_battery }

    /// Retrieves the remaining fuel level of the satellite.
    ///
    /// # Returns
    /// - A `f32` value representing the remaining percentage of fuel.
    pub fn fuel_left(&self) -> f32 { self.fuel_left }

    /// Retrieves the current operational state of the satellite.
    ///
    /// The state of the satellite determines its behavior, such as charging (`Charge`),
    /// image acquisition (`Acquisition`), ...
    ///
    /// # Returns
    /// - A `FlightState` enum denoting the active operational state.
    pub fn state(&self) -> FlightState { self.current_state }

    pub fn client(&self) -> Arc<http_client::HTTPClient> { Arc::clone(&self.request_client) }

    pub async fn reset(&self) {
        ResetRequest {}.send_request(&self.request_client).await.expect("ERROR: Failed to reset");
    }

    /// Waits for a given amount of time with debug prints, this is a static method.
    ///
    /// # Arguments
    /// - `sleep`: The duration for which the system should wait.
    pub async fn wait_for_duration(sleep: Duration) {
        if sleep.as_secs() == 0 {
            println!("[LOG] Wait call rejected! Duration was 0!");
            return;
        }
        println!("[INFO] Waiting for {} seconds!", sleep.as_secs());
        tokio::time::sleep(sleep).await;
    }

    async fn wait_for_condition<F>(
        locked_self: &RwLock<Self>,
        (condition, rationale): (F, String),
        timeout_millis: u16,
        poll_interval: u16,
    ) where
        F: Fn(&Self) -> bool,
    {
        println!("[LOG] Waiting for condition: {rationale}");
        let start_time = std::time::Instant::now();
        while start_time.elapsed().as_millis() < u128::from(timeout_millis) {
            let cond = { condition(&*locked_self.read().await) };
            if cond {
                println!(
                    "[LOG] Condition met after {} ms",
                    start_time.elapsed().as_millis()
                );
                return;
            }
            tokio::time::sleep(Duration::from_millis(u64::from(poll_interval))).await;
        }
        println!("[LOG] Condition not met after {timeout_millis} ms");
    }

    /// Transitions the satellite to a new operational state and waits for transition completion.
    ///
    /// # Arguments
    /// - `locked_self`: A `RwLock<Self>` reference to the active flight computer.
    /// - `new_state`: The target operational state.
    pub async fn set_state_wait(locked_self: &RwLock<Self>, new_state: FlightState) {
        let init_state = { locked_self.read().await.current_state };
        if new_state == init_state {
            println!("[LOG] State already set to {new_state}");
            // return; // TODO: here an error should be returned or logged maybe???
        } else if !Self::LEGAL_TARGET_STATES.contains(&new_state) {
            panic!("[FATAL] State {new_state} is not a legal target state");
            // return; // TODO: here an error should be returned or logged maybe???
        } else if init_state == FlightState::Transition {
            panic!("[FATAL] State cant be changed when in {init_state}");
            // return; // TODO: here an error should be returned or logged or sth.
        }
        locked_self.read().await.set_state(new_state).await;

        let transition_t =
            TRANSITION_DELAY_LOOKUP.get(&(init_state, new_state)).unwrap_or_else(|| {
                panic!("[FATAL] ({init_state}, {new_state}) not in TRANSITION_DELAY_LOOKUP")
            });

        Self::wait_for_duration(*transition_t).await;
        let cond = (
            |cont: &FlightComputer| cont.state() == new_state,
            format!("State equals {new_state}"),
        );
        Self::wait_for_condition(locked_self, cond, Self::DEF_COND_TO, Self::DEF_COND_PI).await;
    }

    /// Adjusts the velocity of the satellite and waits until the target velocity is reached.
    ///
    /// # Arguments
    /// - `locked_self`: A `RwLock<Self>` reference to the active flight computer.
    /// - `new_vel`: The target velocity vector.
    pub async fn set_vel_wait(locked_self: &RwLock<Self>, new_vel: Vec2D<f32>) {
        let (current_state, current_vel) = {
            let f_cont_read = locked_self.read().await;
            (f_cont_read.state(), f_cont_read.current_vel())
        };
        if current_state != FlightState::Acquisition {
            panic!("[FATAL] Velocity cant be changed in state {current_state}");
            // return; // TODO: here an error should be logged or returned
        }
        // TODO: there is a http error in this method
        let vel_change_dt =
            Duration::from_secs_f32(new_vel.to(&current_vel).abs() / Self::ACC_CONST);
        locked_self.read().await.set_vel(new_vel).await;

        Self::wait_for_duration(vel_change_dt).await;
        let comp_new_vel = (new_vel * 100).round();
        let cond = (
            |cont: &FlightComputer| (cont.current_vel() * 100).round() == comp_new_vel,
            format!("Vel equals [{}, {}]", new_vel.x(), new_vel.y()),
        );
        Self::wait_for_condition(locked_self, cond, Self::DEF_COND_TO, Self::DEF_COND_PI).await;
    }

    pub async fn set_angle_wait(locked_self: &RwLock<Self>, new_angle: CameraAngle) {
        let (current_angle, current_state) = {
            let f_cont_read = locked_self.read().await;
            (f_cont_read.current_angle, f_cont_read.state())
        };
        if current_angle == new_angle {
            println!("[LOG] Angle already set to {new_angle}");
            return; // TODO: here an error should be logged or returned
        }
        if current_state != FlightState::Acquisition {
            panic!("[FATAL] Angle cant be changed in state {current_state}");
            // return; // TODO: here an error should be logged or returned
        }

        locked_self.read().await.set_angle(new_angle).await;
        let cond = (
            |cont: &FlightComputer| cont.current_angle() == new_angle,
            format!("Lens equals {new_angle}"),
        );
        Self::wait_for_condition(locked_self, cond, Self::DEF_COND_TO, Self::DEF_COND_PI).await;
    }

    /// Updates the satellite's internal fields with the latest observation data.
    pub async fn update_observation(&mut self) {
        loop {
            match (ObservationRequest {}.send_request(&self.request_client).await) {
                Ok(obs) => {
                    self.current_pos =
                        Vec2D::from((f32::from(obs.pos_x()), f32::from(obs.pos_y())));
                    self.current_vel = Vec2D::from((obs.vel_x(), obs.vel_y()));
                    self.current_state = FlightState::from(obs.state());
                    self.current_angle = CameraAngle::from(obs.angle());
                    self.last_observation_timestamp = obs.timestamp();
                    self.current_battery = obs.battery();
                    self.max_battery = obs.max_battery();
                    self.fuel_left = obs.fuel();
                    return;
                }
                Err(_) => {
                    println!("[ERROR] Unnoticed HTTP Error in updateObservation()");
                    /* TODO: log error here */
                }
            }
        }
    }

    /// Sets the satellite’s `FlightState`.
    ///
    /// # Arguments
    /// - `new_state`: The new operational state.
    async fn set_state(&self, new_state: FlightState) {
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel.x(),
            vel_y: self.current_vel.y(),
            camera_angle: self.current_angle.into(),
            state: new_state.into(),
        };
        loop {
            match req.send_request(&self.request_client).await {
                Ok(_) => {
                    println!("[LOG] State change started to {new_state}");
                    return;
                }
                Err(_) => {
                    println!("[ERROR] Unnoticed HTTP Error in set_state()");
                    /* TODO: log error here */
                }
            }
        }
    }

    /// Sets the satellite’s velocity. The input velocity should only have two decimal places after comma.
    ///
    /// # Arguments
    /// - `new_vel`: The new velocity.
    async fn set_vel(&self, new_vel: Vec2D<f32>) {
        let req = ControlSatelliteRequest {
            vel_x: new_vel.x(),
            vel_y: new_vel.y(),
            camera_angle: self.current_angle.into(),
            state: self.current_state.into(),
        };
        loop {
            match req.send_request(&self.request_client).await {
                Ok(_) => {
                    println!(
                        "[LOG] Velocity change commanded to [{}, {}]",
                        new_vel.x(),
                        new_vel.y()
                    );
                    return;
                }
                Err(_) => {
                    println!("[ERROR] Unnoticed HTTP Error in set_vel()");
                    /* TODO: log error here */
                }
            }
        }
    }

    /// Sets the satellite’s `CameraAngle`
    ///
    /// # Arguments
    /// - `new_angle`: The new Camera Angle.
    async fn set_angle(&self, new_angle: CameraAngle) {
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel.x(),
            vel_y: self.current_vel.y(),
            camera_angle: new_angle.into(),
            state: self.current_state.into(),
        };
        loop {
            match req.send_request(&self.request_client).await {
                Ok(_) => {
                    println!("[LOG] Angle change commanded to {new_angle}");
                    return;
                }
                Err(_) => {
                    println!("[ERROR] Unnoticed HTTP Error in setAngle()"); /* TODO: log error here */
                }
            }
        }
    }

    /// Predicts the satellite’s position after a specified time interval.
    ///
    /// # Arguments
    /// - `time_delta`: The time interval for prediction.
    ///
    /// # Returns
    /// - A `Vec2D<f32>` representing the satellite’s predicted position.
    pub fn pos_in_dt(&self, now: IndexedOrbitPosition, dt: chrono::TimeDelta) -> IndexedOrbitPosition {
        let pos = 
            self.current_pos + (self.current_vel * dt.num_seconds()).wrap_around_map();
        now.new_from_future_pos(pos, dt)
        
        
        
    }
}
