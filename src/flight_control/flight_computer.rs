use super::{
    camera_state::CameraAngle,
    common::Vec2D,
    flight_state::{FlightState, TRANSITION_DELAY_LOOKUP},
};
use crate::http_handler::http_request::configure_simulation_put::ConfigureSimulationRequest;
use crate::http_handler::http_response::control_satellite::ControlSatelliteResponse;
use crate::http_handler::{
    http_client,
    http_request::{
        control_put::*,
        observation_get::*,
        request_common::{JSONBodyHTTPRequestType, NoBodyHTTPRequestType},
    },
    HTTPError,
};
use std::cmp::min;
use std::time;
use tokio::time::sleep;

#[derive(Debug)]
pub struct FlightComputer<'a> {
    current_pos: Vec2D<f64>,
    current_vel: Vec2D<f64>,
    current_state: FlightState,
    current_camera_state: CameraAngle,
    max_battery: f32, // this is an artifact caused by dumb_main
    fuel_left: f32,
    last_observation_timestamp: chrono::DateTime<chrono::Utc>,
    request_client: &'a http_client::HTTPClient,
}

impl<'a> FlightComputer<'a> {
    const ACCELERATION: f32 = 0.04;
    const FAST_FORWARD_SECONDS_THRESHOLD: time::Duration = time::Duration::from_secs(15);
    const MAX_TIMESPEED_MULTIPLIER: i32 = 20;

    pub async fn new(request_client: &'a http_client::HTTPClient) -> FlightComputer<'a> {
        let mut return_controller = FlightComputer {
            current_pos: Vec2D::new(0.0, 0.0),
            current_vel: Vec2D::new(0.0, 0.0),
            current_state: FlightState::Safe,
            current_camera_state: CameraAngle::Normal,
            max_battery: 0.0,
            fuel_left: 0.0,
            last_observation_timestamp: chrono::Utc::now(),
            request_client,
        };
        return_controller.update_observation().await;
        return_controller
    }

    pub fn get_max_battery(&self) -> f32 {
        self.max_battery
    }
    pub fn get_state(&self) -> FlightState {
        self.current_state
    }
    pub fn get_fuel_left(&self) -> f32 {
        self.fuel_left
    }

    pub async fn fast_forward(&self, duration: time::Duration) {
        if duration <= Self::FAST_FORWARD_SECONDS_THRESHOLD {
            sleep(duration).await;
            return;
        }

        let t_normal = 2; // At least 2 second at normal speed
        let wait_time = duration.as_secs() - t_normal;
        let mut selected_factor = 1;
        let mut selected_remainder = 0;
        let selected_saved_time = 0;

        for factor in 1..=min(wait_time, Self::MAX_TIMESPEED_MULTIPLIER as u64) {
            let remainder = wait_time % factor;
            let saved_time = wait_time - factor * (wait_time / factor);
            if selected_saved_time < saved_time {
                selected_factor = factor;
                selected_remainder = remainder;
            }
        }
        ConfigureSimulationRequest {
            is_network_simulation: false,
            user_speed_multiplier: selected_factor as u32,
        }
        .send_request(self.request_client)
        .await
        .unwrap();
        sleep(time::Duration::from_secs(wait_time / selected_factor)).await;
        ConfigureSimulationRequest {
            is_network_simulation: false,
            user_speed_multiplier: 1,
        }
        .send_request(self.request_client)
        .await
        .unwrap();
        sleep(time::Duration::from_secs(selected_remainder + t_normal)).await;
    }

    pub async fn set_state(&mut self, new_state: FlightState) {
        self.update_observation().await;
        if new_state == self.current_state
            || new_state == FlightState::Transition
            || new_state == FlightState::Safe
        {
            return;
        }
        self.current_state = FlightState::Transition;
        self.perform_state_transition(new_state).await;
        sleep(TRANSITION_DELAY_LOOKUP[&(self.current_state, new_state)]).await;
        self.update_observation().await;
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
            Err(_) => { /* TODO: log error here */ }
        }
        self.update_observation().await;
    }

    pub async fn update_observation(&mut self) {
        loop {
            match (ObservationRequest {}
                .send_request(self.request_client)
                .await)
            {
                Ok(observation) => {
                    self.current_pos = Vec2D::from((
                        f64::from(observation.pos_x()),
                        f64::from(observation.pos_y()),
                    ));
                    self.current_vel = Vec2D::from((observation.vel_x(), observation.vel_y()));
                    self.current_state = FlightState::from(observation.state());
                    self.last_observation_timestamp = observation.timestamp();
                    self.max_battery = observation.max_battery();
                    self.fuel_left = observation.fuel();
                    return;
                }
                Err(err) => { /* TODO: log error here */ }
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
                Err(err) => { /* TODO: log error here */ }
            }
        }
    }

    pub async fn rotate_vel(&mut self, angle_degrees: f32) {
        let current_vel = self.current_vel;
        self.current_vel.rotate_by(angle_degrees);
        let time_to_sleep = current_vel.to(&self.current_vel).abs() / f64::from(Self::ACCELERATION);
        loop {
            match (ControlSatelliteRequest {
                vel_x: self.current_vel.x(),
                vel_y: self.current_vel.y(),
                camera_angle: self.current_camera_state.into(),
                state: self.current_state.into(),
            }
            .send_request(self.request_client)
            .await)
            {
                Ok(_) => {
                    self.fast_forward(time::Duration::from_secs((time_to_sleep + 1.0) as u64))
                        .await;
                    self.update_observation().await;
                }
                Err(err) => { /* TODO: log error here */ }
            }
        }
    }

    pub fn pos_in_time_delta(&self, time_delta: chrono::TimeDelta) -> Vec2D<f64> {
        let mut pos = self.current_pos + self.current_vel * time_delta.num_seconds();
        pos.wrap_around_map();
        pos
    }

    pub fn get_current_pos(&self) -> Vec2D<f64> {
        self.current_pos
    }

    /* TODO: not implemented
    fn picture_options_p<T>(&self, point: Vec2D<T>, time_delta: TimeDelta, margin: i16) -> PictureOption
    where
        T: num_traits::NumCast + num_traits::Num,
    {
        let total_disp = self.current_vel * time_delta.num_seconds();
    }
    */
}
