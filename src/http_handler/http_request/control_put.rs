use super::request_common::{HTTPRequestMethod, HTTPRequestType, JSONBodyHTTPRequestType};
use super::control_satellite::ControlSatelliteResponse;

#[derive(serde::Serialize, Debug)]
pub struct ControlSatelliteRequest {
    pub vel_x: f64,
    pub vel_y: f64,
    pub camera_angle: String,
    pub state: String,
}

impl JSONBodyHTTPRequestType for ControlSatelliteRequest {
    type Body = ControlSatelliteRequest;
    fn body(&self) -> &Self::Body { &self }
}

impl HTTPRequestType for ControlSatelliteRequest {
    type Response = ControlSatelliteResponse;
    fn endpoint(&self) -> &str { "/control" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
}