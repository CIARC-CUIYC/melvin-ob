use super::{
    camera_state::CameraAngle,
    common::{pinned_dt::PinnedTimeDelay, vec2d::Vec2D},
    flight_state::{FlightState, TRANSITION_DELAY_LOOKUP},
};
use crate::http_handler::{
    http_client,
    http_request::{
        control_put::*,
        observation_get::*,
        request_common::{JSONBodyHTTPRequestType, NoBodyHTTPRequestType},
        configure_simulation_put::ConfigureSimulationRequest
    },
};
use chrono::TimeDelta;
use std::{cmp::min, time::Duration};
use tokio::time::sleep;

#[derive(Debug)]
pub struct FlightComputer<'a> {
    current_pos: Vec2D<f32>,
    current_vel: Vec2D<f32>,
    current_state: FlightState,
    current_camera_state: CameraAngle,
    current_battery: f32, // this is an artifact caused by dumb main
    max_battery: f32,     // this is an artifact caused by dumb_main
    fuel_left: f32,
    last_observation_timestamp: chrono::DateTime<chrono::Utc>,
    request_client: &'a http_client::HTTPClient,
    next_image: PinnedTimeDelay,
    next_image_valid: bool,
}

impl<'a> FlightComputer<'a> {
    const ACCELERATION: f32 = 0.04;
    const FAST_FORWARD_SECONDS_THRESHOLD: Duration = Duration::from_secs(15);
    const MAX_TIMESPEED_MULTIPLIER: i32 = 20;
    const TIME_DELAY_NO_IMAGE: i64 = -100000;

    pub async fn new(request_client: &'a http_client::HTTPClient) -> FlightComputer<'a> {
        let mut return_controller = FlightComputer {
            current_pos: Vec2D::new(0.0, 0.0),
            current_vel: Vec2D::new(0.0, 0.0),
            current_state: FlightState::Safe,
            current_camera_state: CameraAngle::Normal,
            current_battery: 0.0,
            max_battery: 0.0,
            fuel_left: 0.0,
            last_observation_timestamp: chrono::Utc::now(),
            next_image: PinnedTimeDelay::new(TimeDelta::seconds(Self::TIME_DELAY_NO_IMAGE)),
            next_image_valid: false, // TODO: make this cleaner (enum maybe, combining next_image(_valid))
            request_client,
        };
        return_controller.update_observation().await;
        return_controller
    }

    pub fn get_max_battery(&self) -> f32 { self.max_battery }
    pub fn get_state(&self) -> FlightState { self.current_state }
    pub async fn get_state_update(&mut self) -> FlightState { 
        self.update_observation().await;
        self.current_state 
    }
    pub fn get_fuel_left(&self) -> f32 { self.fuel_left }
    pub fn get_battery(&self) -> f32 { self.current_battery }
    pub fn set_next_image(&mut self, dt: PinnedTimeDelay) {
        self.next_image = dt;
        self.next_image_valid = true;
    }
    pub fn get_next_image(&self) -> PinnedTimeDelay { self.next_image }
    pub fn remove_next_image(&mut self) {
        self.next_image = PinnedTimeDelay::new(TimeDelta::seconds(Self::TIME_DELAY_NO_IMAGE));
        self.next_image_valid = false;
    }

    pub async fn make_ff_call(&mut self, sleep: Duration) {
        if sleep.as_secs() == 0 {
            println!("[LOG] Wait call rejected! Duration was 0!");
            return;
        }
        println!("[INFO] Waiting for {} seconds!", sleep.as_secs());
        self.next_image
            .sub_delay(TimeDelta::seconds(self.fast_forward(sleep).await as i64));
        let time_left = self.next_image.time_left();
        print!("[INFO] Return from Waiting!");
        if (self.next_image_valid) {
            println!(
                " Next Image in {:02}:{:02}:{:02}",
                time_left.num_hours(),
                time_left.num_minutes() - time_left.num_hours() * 60,
                time_left.num_seconds() - time_left.num_minutes() * 60
            );
        }else { println!(); }
    }

    async fn fast_forward(&self, duration: Duration) -> u64 {
        if duration <= Self::FAST_FORWARD_SECONDS_THRESHOLD {
            sleep(duration - Duration::from_secs(1)).await;
            return 0;
        }

        let t_normal = 4; // At least 2 second at normal speed
        let wait_time = duration.as_secs() - t_normal;
        let mut selected_factor = 1;
        let mut selected_remainder = 0;
        let mut selected_saved_time = 0;

        for factor in 1..=min(wait_time, Self::MAX_TIMESPEED_MULTIPLIER as u64) {
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
                user_speed_multiplier: selected_factor as u32,
            }.send_request(self.request_client).await;
            match resp{
                Ok(..) => break,
                Err(e) => println!("[ERROR] {e} while starting fast-forward."),
            };// TODO: HTTP Error here
        }
        sleep(Duration::from_secs(
            (wait_time - selected_remainder) / selected_factor,
        ))
        .await;
        loop {
            let resp = ConfigureSimulationRequest {
                is_network_simulation: false,
                user_speed_multiplier: 1,
            }.send_request(self.request_client).await;
           match resp{
                Ok(..) => break,
                Err(e) => println!("[ERROR] {e} while returning from fast-forward."),
            };// TODO: HTTP Error here
        }
        sleep(Duration::from_secs(selected_remainder)).await;
        selected_saved_time
    }

    pub async fn charge_until(&mut self, target_battery: f32) {
        let charge_rate: f32  = FlightState::Charge.get_charge_rate();
        let charge_time_s = ((target_battery - self.current_battery) / charge_rate) * 2.0;
        let charge_time = Duration::from_secs_f32(charge_time_s);
        let initial_state = self.current_state;
        self.set_state(FlightState::Charge).await;
        self.make_ff_call(charge_time).await;
        self.set_state(initial_state).await;
        self.update_observation().await;
    }

    pub async fn set_state(&mut self, new_state: FlightState) {
        self.update_observation().await;
        if new_state == self.current_state
            || new_state == FlightState::Transition
            || new_state == FlightState::Safe
        {
            return;
        }
        let init_state = self.current_state;
        self.perform_state_transition(new_state).await;
        let transition_t = TRANSITION_DELAY_LOOKUP
            .get(&(init_state, new_state))
            .unwrap_or_else(||{panic!("[FATAL] ({init_state}, {new_state}) not in TRANSITION_DELAY_LOOKUP")});
        self.make_ff_call(*transition_t)
            .await;
        // TODO: implement "wait_for_condition" functionality with condition enum
        self.update_observation().await;
        while self.current_state != new_state {
            sleep(Duration::from_millis(100)).await;
            self.update_observation().await;
        }
    }

    pub async fn set_angle(&mut self, new_angle: CameraAngle) {
        self.update_observation().await;
        if new_angle == self.current_camera_state {
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
                self.current_camera_state = new_angle;
            }
            Err(_) => {
                println!("[ERROR] Unnotized HTTP Error in setAngle()"); /* TODO: log error here */
            }
        }
        self.update_observation().await;
    }

    pub async fn update_observation(&mut self) {
        loop {
            match (ObservationRequest {}
                .send_request(self.request_client)
                .await)
            {
                Ok(obs) => {
                    self.current_pos =
                        Vec2D::from((obs.pos_x() as f32, obs.pos_y() as f32));
                    self.current_vel = Vec2D::from((obs.vel_x() as f32, obs.vel_y() as f32));
                    self.current_state = FlightState::from(obs.state());
                    self.last_observation_timestamp = obs.timestamp();
                    self.current_battery = obs.battery();
                    self.max_battery = obs.max_battery();
                    self.fuel_left = obs.fuel();
                    return;
                }
                Err(err) => {
                    println!("[ERROR] Unnotized HTTP Error in updateObservation()"); /* TODO: log error here */
                }
            }
        }
    }

    async fn perform_state_transition(&mut self, new_state: FlightState) {
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel.x(),
            vel_y: self.current_vel.y(),
            camera_angle: self.current_camera_state.into(),
            state: new_state.into(),
        };
        loop {
            match req.send_request(self.request_client).await {
                Ok(_) => {
                    return;
                }
                Err(err) => {
                    println!("[ERROR] Unnotized HTTP Error in perform_state_transition()")
                    /* TODO: log error here */
                }
            }
        }
    }

    pub async fn rotate_vel(&mut self, angle_degrees: f32, accel_factor: f32) {
        self.update_observation().await;
        // TODO: there is a http error in this method
        if self.current_state != FlightState::Acquisition {
            self.set_state(FlightState::Acquisition).await;
        }
        let current_vel = self.current_vel;
        self.current_vel.rotate_by(angle_degrees);
        self.current_vel = self.current_vel * (1.0 + accel_factor);
        let time_to_sleep = (current_vel.to(&self.current_vel).abs() / Self::ACCELERATION)*2.0;

        loop {
            let req = ControlSatelliteRequest {
                vel_x: self.current_vel.x(),
                vel_y: self.current_vel.y(),
                camera_angle: self.current_camera_state.into(),
                state: self.current_state.into(),
            };
            match req.send_request(self.request_client).await {
                Ok(_) => {
                    self.make_ff_call(Duration::from_secs((time_to_sleep + 1.0) as u64))
                        .await;
                    self.update_observation().await;
                    break;
                }
                Err(err) => {
                    println!("[ERROR] Unnotized HTTP Error in rotate_vel") /* TODO: log error here */
                }
            }
        }
    }

    pub fn pos_in_time_delta(&self, time_delta: TimeDelta) -> Vec2D<f32> {
        let mut pos = self.current_pos + (self.current_vel * time_delta.num_seconds());
        pos.wrap_around_map();
        pos
    }

    pub fn get_current_pos(&self) -> Vec2D<f32> { self.current_pos }

    /* TODO: not implemented
    fn picture_options_p<T>(&self, point: Vec2D<T>, time_delta: TimeDelta, margin: i16) -> PictureOption
    where
        T: num_traits::NumCast + num_traits::Num,
    {
        let total_disp = self.current_vel * time_delta.num_seconds();
    }
    */
}
