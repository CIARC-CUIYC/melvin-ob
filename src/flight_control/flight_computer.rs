use chrono::TimeDelta;
use tokio::time::sleep;
use super::{flight_state::{FlightState, TRANSITION_DELAY_LOOKUP},
            camera_state::CameraAngle, common::Vec2D};
use crate::http_handler::{http_client, http_request::{control_put::*, observation_get::*,
                                                      request_common::{NoBodyHTTPRequestType,
                                                                       JSONBodyHTTPRequestType}}};

#[derive(Debug)]
pub struct FlightComputer<'a> {
    current_pos: Vec2D<f64>,
    current_vel: Vec2D<f64>,
    current_state: FlightState,
    current_camera_state: CameraAngle,
    last_observation_timestamp: chrono::DateTime<chrono::Utc>,
    request_client: &'a http_client::HTTPClient,
}

impl<'a> FlightComputer<'a> {
    pub async fn new(request_client: &'a http_client::HTTPClient) -> FlightComputer<'a> {
        let mut return_controller = FlightComputer {
            current_pos: Vec2D::new(0.0, 0.0),
            current_vel: Vec2D::new(0.0, 0.0),
            current_state: FlightState::Safe,
            current_camera_state: CameraAngle::Normal,
            last_observation_timestamp: chrono::Utc::now(),
            request_client,
        };
        return_controller.update_observation().await;
        return_controller
    }

    fn get_state(&self) -> &FlightState { &self.current_state }

    pub async fn set_state(&mut self, new_state: FlightState) {
        self.update_observation().await;
        if new_state == self.current_state ||
            new_state == FlightState::Transition ||
            new_state == FlightState::Safe
        { return; }
        self.current_state = FlightState::Transition;
        self.perform_state_transition(new_state).await;
        sleep(TRANSITION_DELAY_LOOKUP[&(self.current_state, new_state)]).await;
        self.update_observation().await;
    }

    async fn update_observation(&mut self) {
        loop {
            match (ObservationRequest {}.send_request(self.request_client).await) {
                Ok(observation) => {
                    self.current_pos = Vec2D::from((observation.pos_x() as f64, observation.pos_y() as f64));
                    self.current_vel = Vec2D::from((observation.vel_x(), observation.vel_y()));
                    self.current_state = FlightState::from(observation.state());
                    self.last_observation_timestamp = observation.timestamp();
                    return;
                }
                Err(err) => { println!("{}", err); }
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

    pub fn pos_in_time_delta(&self, time_delta: TimeDelta) -> Vec2D<f64> {
        let mut pos = self.current_pos + self.current_vel * time_delta.num_seconds();
        pos.wrap_around_map();
        pos
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