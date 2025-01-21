use super::http_request::request_common::RequestError;
use super::http_response::response_common::ResponseError;
use crate::flight_control::camera_state::CameraAngle;
use chrono::Duration;
use strum_macros::Display;

#[derive(serde::Deserialize, serde::Serialize, Debug, Clone)]
pub struct ZonedObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    decrease_rate: i64,
    enabled: bool,
    zone: [i32; 4],
    // TODO: make an enum out of optic_required
    optic_required: CameraAngle,
    coverage_required: usize,
    description: String,
    sprite: String,
    secret: bool,
}

impl ZonedObjective {
    pub fn id(&self) -> usize {
        self.id
    }
    fn name(&self) -> &str {
        &self.name
    }
    fn decrease_rate(&self) -> i64 {
        self.decrease_rate
    }
    fn is_enabled(&self) -> bool {
        self.enabled
    }
    pub fn zone(&self) -> &[i32; 4] {
        &self.zone
    }
    pub fn optic_required(&self) -> &CameraAngle {
        &self.optic_required
    }
    pub fn coverage_required(&self) -> usize {
        self.coverage_required
    }
    fn sprite(&self) -> &str {
        &self.sprite
    }
    fn is_secret(&self) -> bool {
        self.secret
    }
}

impl Timed for ZonedObjective {
    fn start(&self) -> chrono::DateTime<chrono::Utc> {
        self.start
    }
    fn end(&self) -> chrono::DateTime<chrono::Utc> {
        self.end
    }
}

#[derive(serde::Deserialize, serde::Serialize, Debug)]
pub struct BeaconObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
}

impl BeaconObjective {
    fn name(&self) -> &str {
        self.name.as_str()
    }
    fn id(&self) -> usize {
        self.id
    }
}

impl Timed for BeaconObjective {
    fn start(&self) -> chrono::DateTime<chrono::Utc> {
        self.start
    }
    fn end(&self) -> chrono::DateTime<chrono::Utc> {
        self.end
    }
}

#[derive(serde::Deserialize, Debug)]
pub struct CommunicationSlot {
    id: usize,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    enabled: bool,
}

impl CommunicationSlot {
    fn is_enabled(&self) -> bool {
        self.enabled
    }
    fn id(&self) -> usize {
        self.id
    }
}

impl Timed for CommunicationSlot {
    fn start(&self) -> chrono::DateTime<chrono::Utc> {
        self.start
    }
    fn end(&self) -> chrono::DateTime<chrono::Utc> {
        self.end
    }
}

#[derive(serde::Deserialize, Debug)]
pub struct Achievement {
    name: String,
    done: bool,
    points: f32,
    description: String,
    goal_parameter_threshold: bool,
    goal_parameter: bool,
}

impl Achievement {
    fn name(&self) -> &str {
        &self.name
    }
    fn is_done(&self) -> bool {
        self.done
    }
    fn points(&self) -> f32 {
        self.points
    }
    fn description(&self) -> &str {
        &self.description
    }
    fn is_goal_parameter_threshold(&self) -> bool {
        self.goal_parameter_threshold
    }
    fn is_goal_parameter(&self) -> bool {
        self.goal_parameter
    }
}

pub trait Timed {
    fn start(&self) -> chrono::DateTime<chrono::Utc>;
    fn end(&self) -> chrono::DateTime<chrono::Utc>;

    fn time_to_start(&self) -> Option<chrono::Duration> {
        let now = chrono::Utc::now();
        if now < self.start() {
            Some(self.start() - now)
        } else {
            None
        }
    }

    fn time_to_end(&self) -> Option<chrono::Duration> {
        let now = chrono::Utc::now();
        if now < self.end() {
            Some(self.end() - now)
        } else {
            None
        }
    }

    fn is_in_time_window(&self) -> bool {
        chrono::Utc::now() >= self.start() && chrono::Utc::now() <= self.end()
    }

    fn is_in_future_time_window(&self, time_in_future: i64) -> bool {
        let time_in_future = chrono::Utc::now() + Duration::seconds(time_in_future);

        time_in_future >= self.start() && time_in_future <= self.end()
    }
}

#[derive(Debug, Display)]
pub enum HTTPError {
    HTTPRequestError(RequestError),
    HTTPResponseError(ResponseError),
}

impl std::error::Error for HTTPError {}
