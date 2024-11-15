use std::collections::HashMap;
use std::sync::LazyLock;
use std::time::Duration;
use tokio::time::sleep;
use super::flight_state::FlightState;
use super::camera_state::CameraAngle;
use crate::http_handler::{http_client, http_request::{control_put::ControlSatelliteRequest,
                                                      observation_get::ObservationRequest,
                                                      request_common::{NoBodyHTTPRequestType,
                                                                       JSONBodyHTTPRequestType}}};

#[derive(Debug)]
struct FlightComputer<'a> {
    current_pos_x: f64,
    current_pos_y: f64,
    current_vel_x: f64,
    current_vel_y: f64,
    current_state: FlightState,
    current_camera_state: CameraAngle,
    last_observation_timestamp: chrono::DateTime<chrono::Utc>,
    request_client: &'a http_client::HTTPClient,
}

impl<'a> FlightComputer<'a> {
    fn get_state(&self) -> &FlightState { &self.current_state }
    
    async fn set_state(&mut self, new_state: FlightState) {
        self.update_observation().await;
        if new_state == self.current_state ||
            new_state == FlightState::Transition ||
            new_state == FlightState::Safe
        { return; }
        self.current_state = FlightState::Transition;
        self.perform_state_transition(&new_state).await;
        sleep(TRANSITION_DELAY_LOOKUP[&(self.current_state, new_state)]).await;
        self.update_observation().await;
    }

    async fn update_observation(&mut self) {
        loop {
            match (ObservationRequest {}.send_request(self.request_client).await) {
                Ok(observation) => {
                    self.current_pos_x = observation.pos_x();
                    self.current_pos_y = observation.pos_y();
                    self.current_vel_x = observation.vel_x();
                    self.current_vel_y = observation.vel_y();
                    self.current_state = FlightState::from(observation.state());
                    self.last_observation_timestamp = observation.timestamp();
                }
                Err(err) => { /* TODO: log error here */ }
            }
        }
    }

    async fn perform_state_transition(&mut self, new_state: &FlightState) {
        let req = ControlSatelliteRequest {
            vel_x: self.current_vel_x,
            vel_y: self.current_vel_y,
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
}

static TRANSITION_DELAY_LOOKUP: LazyLock<HashMap<(FlightState, FlightState), Duration>> =
    LazyLock::new(|| {
        let mut lookup = HashMap::new();
        let transition_times = vec![
            // Deployment transitions
            (FlightState::Deployment, FlightState::Acquisition, Duration::from_secs(180)),
            (FlightState::Deployment, FlightState::Charge, Duration::from_secs(180)),
            (FlightState::Deployment, FlightState::Comms, Duration::from_secs(180)),
            // Acquisition transitions
            (FlightState::Acquisition, FlightState::Deployment, Duration::from_secs(180)),
            (FlightState::Acquisition, FlightState::Charge, Duration::from_secs(180)),
            (FlightState::Acquisition, FlightState::Comms, Duration::from_secs(180)),
            // Charge transitions
            (FlightState::Charge, FlightState::Deployment, Duration::from_secs(180)),
            (FlightState::Charge, FlightState::Acquisition, Duration::from_secs(180)),
            (FlightState::Charge, FlightState::Comms, Duration::from_secs(180)),
            // Comms transitions
            (FlightState::Comms, FlightState::Deployment, Duration::from_secs(180)),
            (FlightState::Comms, FlightState::Acquisition, Duration::from_secs(180)),
            (FlightState::Comms, FlightState::Charge, Duration::from_secs(180)),
            // Safe transitions
            (FlightState::Safe, FlightState::Deployment, Duration::from_secs(1200)),
            (FlightState::Safe, FlightState::Acquisition, Duration::from_secs(1200)),
            (FlightState::Safe, FlightState::Charge, Duration::from_secs(1200)),
        ];

        for (from, to, duration) in transition_times {
            lookup.insert((from, to), duration);
        }
        lookup
    });
