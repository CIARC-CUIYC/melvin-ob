use super::{
    camera_state::CameraAngle,
    common::{pinned_dt::PinnedTimeDelay, vec2d::Vec2D},
    flight_state::{FlightState, TRANSITION_DELAY_LOOKUP},
};
use crate::http_handler::{
    http_client,
    http_request::{
        configure_simulation_put::ConfigureSimulationRequest,
        control_put::ControlSatelliteRequest,
        observation_get::ObservationRequest,
        request_common::{JSONBodyHTTPRequestType, NoBodyHTTPRequestType},
    },
};
use chrono::TimeDelta;
use std::collections::VecDeque;
use std::{cmp::min, time::Duration};
use tokio::time::sleep;

/// Represents the core flight computer for satellite control.
/// It manages operations such as state changes, velocity updates,
/// battery charging, and fast-forward simulation.
///
/// This system interfaces with an external HTTP-based control system to
/// send requests for managing the satellite’s behavior (e.g., velocity updates,
/// state transitions). Additionally, the file schedules satellite-related
/// activities like image captures and ensures the system stays updated
/// using observations.
///
/// Key methods allow high-level control, including fast-forward simulations,
/// state transitions, camera angle adjustments, and battery-related tasks.
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
/// - `next_image_schedule`: A list of scheduled image captures.
#[derive(Debug)]
pub struct FlightComputer<'a> {
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
    request_client: &'a http_client::HTTPClient,
    /// Schedule for the next images, represented by time delays.
    next_image_schedule: VecDeque<PinnedTimeDelay>,
}

impl<'a> FlightComputer<'a> {
    /// Acceleration factor for calculations involving velocity and state transitions.
    const ACCELERATION: f32 = 0.04;
    /// Minimum threshold duration for triggering normal-speed fast-forward operations.
    const FAST_FORWARD_SECONDS_THRESHOLD: Duration = Duration::from_secs(15);
    /// Maximum multiplier allowed during simulation's time speed adjustments.
    const MAX_TIME_SPEED_MULTIPLIER: i32 = 20;

    /// Initializes a new `FlightComputer` instance.
    ///
    /// # Arguments
    /// - `request_client`: A reference to the HTTP client for sending simulation requests.
    ///
    /// # Returns
    /// A fully initialized `FlightComputer` with up-to-date field values.
    pub async fn new(request_client: &'a http_client::HTTPClient) -> FlightComputer<'a> {
        let mut return_controller = FlightComputer {
            current_pos: Vec2D::new(0.0, 0.0),
            current_vel: Vec2D::new(0.0, 0.0),
            current_state: FlightState::Safe,
            current_angle: CameraAngle::Normal,
            current_battery: 0.0,
            max_battery: 0.0,
            fuel_left: 0.0,
            last_observation_timestamp: chrono::Utc::now(),
            next_image_schedule: VecDeque::new(),
            request_client,
        };
        return_controller.update_observation().await;
        return_controller
    }

    /// Retrieves the current position of the satellite.
    ///
    /// # Returns
    /// A `Vec2D` representing the current satellite position.
    pub fn current_pos(&self) -> Vec2D<f32> {
        self.current_pos
    }

    /// Retrieves the current position of the velocity.
    ///
    /// # Returns
    /// A `Vec2D` representing the current satellite velocity.
    pub fn current_vel(&self) -> Vec2D<f32> {
        self.current_vel
    }

    /// Retrieves the maximum battery capacity of the satellite.
    ///
    /// This value fluctuates only due to battery depletion safe mode events.
    ///
    /// # Returns
    /// - A `f32` value representing the maximum battery charge.
    pub fn max_battery(&self) -> f32 {
        self.max_battery
    }

    /// Retrieves the current battery charge level of the satellite.
    ///
    /// # Returns
    /// - A `f32` value denoting the battery's current charge level.
    pub fn current_battery(&self) -> f32 {
        self.current_battery
    }

    /// Retrieves the remaining fuel level of the satellite.
    ///
    /// # Returns
    /// - A `f32` value representing the remaining percentage of fuel.
    pub fn fuel_left(&self) -> f32 {
        self.fuel_left
    }

    /// Retrieves the current operational state of the satellite.
    ///
    /// The state of the satellite determines its behavior, such as charging (`Charge`),
    /// image acquisition (`Acquisition`), ...
    ///
    /// # Returns
    /// - A `FlightState` enum denoting the active operational state.
    pub fn state(&self) -> FlightState {
        self.current_state
    }

    /// Adds a new image capture task to the scheduling system.
    ///
    /// # Arguments
    /// - `dt`: A `PinnedTimeDelay` value indicating the delay before the next capture.
    pub fn schedule_image(&mut self, dt: PinnedTimeDelay) {
        self.next_image_schedule.push_back(dt);
    }

    /// Provides a reference to the next scheduled image capture.
    ///
    /// # Panics
    /// - This function panics if there are no scheduled images.
    ///
    /// # Returns
    /// - A reference to the first `PinnedTimeDelay` in the schedule.
    pub fn next_img_ref(&self) -> &PinnedTimeDelay {
        self.next_image_schedule.front().unwrap()
    }

    /// Removes the next scheduled image capture from the schedule.
    pub fn remove_next_image(&mut self) {
        self.next_image_schedule.pop_front();
    }

    /// Fast-forwards the simulation or waits for a specified duration.
    ///
    /// Adjusts delays for all scheduled images in `next_image_schedule` after waiting.
    ///
    /// # Arguments
    /// - `sleep`: The duration for which the system should wait or fast-forward.
    pub async fn make_ff_call(&mut self, sleep: Duration) {
        if sleep.as_secs() == 0 {
            println!("[LOG] Wait call rejected! Duration was 0!");
            return;
        }
        println!("[INFO] Waiting for {} seconds!", sleep.as_secs());
        let slept_secs = i64::from(self.fast_forward(sleep).await);
        self.next_image_schedule.iter_mut().for_each(|dt| {
            dt.sub_delay(TimeDelta::seconds(slept_secs));
        });
        print!("[INFO] Return from Waiting!");
        if self.next_image_schedule.is_empty() {
            println!();
        } else {
            let time_left = self.next_img_ref().time_left();
            println!(
                " Next Image in {:02}:{:02}:{:02}",
                time_left.num_hours(),
                time_left.num_minutes() - time_left.num_hours() * 60,
                time_left.num_seconds() - time_left.num_minutes() * 60
            );
        }
    }

    /// Handles fast-forwarding for the simulation using optimal multipliers.
    ///
    /// Chooses the optimal speed multiplier depending on the duration.
    ///
    /// # Arguments
    /// - `duration`: The total time duration to simulate.
    ///
    /// # Returns
    /// - The amount of time (in seconds) saved by fast-forwarding.
    async fn fast_forward(&self, duration: Duration) -> u32 {
        if duration <= Self::FAST_FORWARD_SECONDS_THRESHOLD {
            sleep(duration - Duration::from_secs(1)).await;
            return 0;
        }

        let t_normal = 4; // At least 2 second at normal speed
        let wait_time = u32::try_from(duration.as_secs() - t_normal).unwrap_or(0);
        let mut selected_factor = 1;
        let mut selected_remainder = 0;
        let mut selected_saved_time = 0;

        for factor in 1..=min(wait_time, Self::MAX_TIME_SPEED_MULTIPLIER as u32) {
            let remainder = wait_time % factor;
            let saved_time = (wait_time - remainder) - ((wait_time - remainder) / factor);
            if selected_saved_time < saved_time {
                selected_factor = factor;
                selected_remainder = remainder;
                selected_saved_time = saved_time;
            }
        }
        println!("[LOG] Fast-Forward with Factor: {selected_factor}");
        loop {
            let resp = ConfigureSimulationRequest {
                is_network_simulation: false,
                user_speed_multiplier: selected_factor,
            }
            .send_request(self.request_client)
            .await;
            match resp {
                Ok(..) => break,
                Err(e) => println!("[ERROR] {e} while starting fast-forward."),
            }; // TODO: HTTP Error here
        }
        let sleep_dur = u64::from((wait_time - selected_remainder) / selected_factor);
        sleep(Duration::from_secs(sleep_dur)).await;
        loop {
            let resp = ConfigureSimulationRequest {
                is_network_simulation: false,
                user_speed_multiplier: 1,
            }
            .send_request(self.request_client)
            .await;
            match resp {
                Ok(..) => break,
                Err(e) => println!("[ERROR] {e} while returning from fast-forward."),
            }; // TODO: HTTP Error here
        }
        sleep(Duration::from_secs(u64::from(selected_remainder))).await;
        selected_saved_time
    }

    /// Charges the satellite battery to a target charge percentage.
    ///
    /// Calculates the charging duration based on the charge rate, executes the charging process and
    /// restores the previous `FlightState` afterward.
    ///
    /// # Arguments
    /// - `target_battery`: The desired target battery level (percentage).
    pub async fn charge_until(&mut self, target_battery: f32) {
        let charge_rate: f32 = FlightState::Charge.get_charge_rate();
        let charge_time_s = ((target_battery - self.current_battery) / charge_rate) * 2.0;
        let charge_time = Duration::from_secs_f32(charge_time_s);
        let initial_state = self.current_state;
        self.state_change_ff(FlightState::Charge).await;
        self.make_ff_call(charge_time).await;
        self.state_change_ff(initial_state).await;
        self.update_observation().await;
    }

    /// Transitions the satellite to a new operational state and fast-forwards during the transition.
    ///
    /// # Arguments
    /// - `new_state`: The target operational state.
    ///
    /// # Panics
    /// - If the transition delay is not defined in `TRANSITION_DELAY_LOOKUP`.
    pub async fn state_change_ff(&mut self, new_state: FlightState) {
        self.update_observation().await;
        if new_state == self.current_state
            || new_state == FlightState::Transition
            || new_state == FlightState::Safe
        {
            return;
        }
        let init_state = self.current_state;
        self.set_state(new_state).await;
        let transition_t = TRANSITION_DELAY_LOOKUP
            .get(&(init_state, new_state))
            .unwrap_or_else(|| {
                panic!("[FATAL] ({init_state}, {new_state}) not in TRANSITION_DELAY_LOOKUP")
            });
        self.make_ff_call(*transition_t).await;
        // TODO: implement "wait_for_condition" functionality with condition enum
        self.update_observation().await;
        while self.current_state != new_state {
            sleep(Duration::from_millis(100)).await;
            self.update_observation().await;
        }
    }

    /// Changes the angle of the satellite's camera.
    ///
    /// # Arguments
    /// - `new_angle`: The new camera angle to set.
    pub async fn set_angle(&mut self, new_angle: CameraAngle) {
        self.update_observation().await;
        if new_angle == self.current_angle {
            return;
        }
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel.x(),
            vel_y: self.current_vel.y(),
            camera_angle: new_angle.into(),
            state: self.current_state.into(),
        };
        match req.send_request(self.request_client).await {
            Ok(_) => {
                self.current_angle = new_angle;
            }
            Err(_) => {
                println!("[ERROR] Unnoticed HTTP Error in setAngle()"); /* TODO: log error here */
            }
        }
        self.update_observation().await;
    }

    /// Adjusts the velocity of the satellite and optionally restores the previous state.
    ///
    /// # Arguments
    /// - `new_vel`: The target velocity vector.
    /// - `switch_back_to_init_state`: Whether to revert to the initial state after adjusting velocity.
    pub async fn set_vel(&mut self, new_vel: Vec2D<f32>, switch_back_to_init_state: bool) {
        let init_state = self.current_state;
        if init_state != FlightState::Acquisition {
            self.state_change_ff(FlightState::Acquisition).await;
        }
        // TODO: there is a http error in this method
        let dv = new_vel.to(&self.current_vel).abs();
        let sleep_time = dv / Self::ACCELERATION * 2.0;
        self.update_observation().await;
        loop {
            let req = ControlSatelliteRequest {
                vel_x: new_vel.x(),
                vel_y: new_vel.y(),
                camera_angle: self.current_angle.into(),
                state: self.current_state.into(),
            };
            match req.send_request(self.request_client).await {
                Ok(_) => {
                    self.make_ff_call(Duration::from_secs_f32(sleep_time + 1.0))
                        .await;
                    self.update_observation().await;
                    break;
                }
                Err(_) => {
                    println!("[ERROR] Unnoticed HTTP Error in rotate_vel"); /* TODO: log error here */
                }
            }
            if switch_back_to_init_state {
                self.state_change_ff(init_state).await;
            }
        }
    }

    /// Updates the satellite's internal fields with the latest observation data.
    pub async fn update_observation(&mut self) {
        loop {
            match (ObservationRequest {}
                .send_request(self.request_client)
                .await)
            {
                Ok(obs) => {
                    self.current_pos =
                        Vec2D::from((f32::from(obs.pos_x()), f32::from(obs.pos_y())));
                    self.current_vel = Vec2D::from((obs.vel_x(), obs.vel_y()));
                    self.current_state = FlightState::from(obs.state());
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
    async fn set_state(&mut self, new_state: FlightState) {
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel.x(),
            vel_y: self.current_vel.y(),
            camera_angle: self.current_angle.into(),
            state: new_state.into(),
        };
        loop {
            match req.send_request(self.request_client).await {
                Ok(_) => {
                    return;
                }
                Err(_) => {
                    println!("[ERROR] Unnoticed HTTP Error in perform_state_transition()");
                    /* TODO: log error here */
                }
            }
        }
    }

    /// Rotates the satellite’s velocity vector by a given angle.
    ///
    /// # Arguments
    /// - `angle_degrees`: The angle (in degrees) by which to rotate the velocity vector.
    /// - `accel_factor`: A scalar factor to apply to accelerate the satellite.
    pub async fn rotate_vel(&mut self, angle_degrees: f32, accel_factor: f32) {
        self.set_vel(
            self.current_vel.rotate_by(angle_degrees) * (1.0 + accel_factor),
            true,
        )
        .await;
    }

    /// Predicts the satellite’s position after a specified time interval.
    ///
    /// # Arguments
    /// - `time_delta`: The time interval for prediction.
    ///
    /// # Returns
    /// - A `Vec2D<f32>` representing the satellite’s predicted position.
    pub fn pos_in_dt(&self, time_delta: TimeDelta) -> Vec2D<f32> {
        self.current_pos + (self.current_vel * time_delta.num_seconds()).wrap_around_map()
    }
}
