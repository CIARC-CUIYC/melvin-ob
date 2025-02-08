use super::http_request::request_common::RequestError;
use super::http_response::response_common::ResponseError;
use crate::flight_control::{camera_state::CameraAngle, common::vec2d::Vec2D};
use fixed::types::I32F32;
use num::ToPrimitive;
use strum_macros::Display;

#[derive(serde::Deserialize, serde::Serialize, Debug, Clone)]
#[serde(untagged)]
pub enum ZoneType {
    KnownZone([i32; 4]),
    SecretZone(String)
}

#[derive(serde::Deserialize, serde::Serialize, Debug, Clone)]
pub struct ImageObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    decrease_rate: f32,
    zone: ZoneType,
    // TODO: make an enum out of optic_required
    optic_required: String,
    coverage_required: f32,
    description: String,
    sprite: Option<String>,
    secret: bool,
}

impl ImageObjective {

    pub fn id(&self) -> usize { self.id }
    pub fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
    pub fn name(&self) -> &str { &self.name }
    pub fn decrease_rate(&self) -> f32 { self.decrease_rate }
    pub fn zone_type(&self) -> &ZoneType { &self.zone }
    pub fn optic_required(&self) -> &str { &self.optic_required }
    pub fn coverage_required(&self) -> f32 { self.coverage_required }
    pub fn sprite(&self) -> Option<&String> { self.sprite.as_ref() }
    pub fn is_secret(&self) -> bool { self.secret }
}

#[derive(serde::Deserialize, serde::Serialize, Debug)]
pub struct BeaconObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    decrease_rate: f32,
    attempts_made: u32,
    description: String,
}

impl BeaconObjective {
    fn name(&self) -> &str { self.name.as_str() }
    fn id(&self) -> usize { self.id }
    fn attempts_made(&self) -> u32 { self.attempts_made }
    fn decrease_rate(&self) -> f32 { self.decrease_rate }
    fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
    fn description(&self) -> &str { self.description.as_str() }
}

#[derive(serde::Deserialize, Debug)]
pub struct CommunicationSlot {
    id: usize,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    enabled: bool,
}

impl CommunicationSlot {
    fn is_enabled(&self) -> bool { self.enabled }
    fn id(&self) -> usize { self.id }
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
    fn name(&self) -> &str { &self.name }
    fn is_done(&self) -> bool { self.done }
    fn points(&self) -> f32 { self.points }
    fn description(&self) -> &str { &self.description }
    fn is_goal_parameter_threshold(&self) -> bool { self.goal_parameter_threshold }
    fn is_goal_parameter(&self) -> bool { self.goal_parameter }
}

#[derive(Debug, Display)]
pub enum HTTPError {
    HTTPRequestError(RequestError),
    HTTPResponseError(ResponseError),
}

impl std::error::Error for HTTPError {}
