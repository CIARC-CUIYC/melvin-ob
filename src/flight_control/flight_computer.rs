use super::{
    camera_state::CameraAngle,
    common::Vec2D,
    flight_state::{FlightState, TRANSITION_DELAY_LOOKUP},
};
use crate::http_handler::{
    http_client,
    http_request::{
        control_put::*,
        observation_get::*,
        request_common::{JSONBodyHTTPRequestType, NoBodyHTTPRequestType},
    },
};
use tokio::time::sleep;
use std::time::Duration;

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

    pub fn get_max_battery(&self) -> f32 { self.max_battery }
    pub fn get_state(&self) -> FlightState { self.current_state }
    pub fn get_fuel_left(&self) -> f32 { self.fuel_left }

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

    pub async fn update_observation(&mut self) {
        loop {
            match (ObservationRequest {}
                .send_request(self.request_client)
                .await)
            {
                Ok(observation) => {
                    self.current_pos = Vec2D::from((observation.pos_x() as f64, observation.pos_y() as f64));
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
            state: (new_state).into(),
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
        let time_to_sleep = (current_vel.to(&self.current_vel).abs() ).abs() / Self::ACCELERATION as f64;
        loop {
            match (ControlSatelliteRequest {
                vel_x: self.current_vel.x(),
                vel_y: self.current_vel.y(),
                camera_angle: self.current_camera_state.into(),
                state: self.current_state.into(),
            }.send_request(self.request_client).await) {
                Ok(_) => {
                    sleep(Duration::from_secs((time_to_sleep + 1.0) as u64)).await;
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
    
