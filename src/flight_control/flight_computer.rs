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
use chrono::TimeDelta;
use tokio::time::sleep;

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
    fn get_state(&self) -> &FlightState {
        &self.current_state
    }

    async fn set_state(&mut self, new_state: FlightState) {
        self.update_observation().await;
        if new_state == self.current_state
            || new_state == FlightState::Transition
            || new_state == FlightState::Safe
        {
            return;
        }
        self.current_state = FlightState::Transition;
        self.perform_state_transition(&new_state).await;
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
                    self.current_pos = Vec2D::from((observation.pos_x(), observation.pos_y()));
                    self.current_vel = Vec2D::from((observation.vel_x(), observation.vel_y()));
                    self.current_state = FlightState::from(observation.state());
                    self.last_observation_timestamp = observation.timestamp();
                }
                Err(err) => { /* TODO: log error here */ }
            }
        }
    }

    async fn perform_state_transition(&mut self, new_state: &FlightState) {
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel.x(),
            vel_y: self.current_vel.y(),
            camera_angle: self.current_camera_state.into(),
            state: (*new_state).into(),
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

    fn pos_in_time_delta(&self, time_delta: TimeDelta) -> Vec2D<f64> {
        self.current_pos + self.current_vel * time_delta.num_seconds()
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
