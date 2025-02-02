use super::control_satellite::ControlSatelliteResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, JSONBodyHTTPRequestType};
use fixed::types::I32F32;

#[derive(serde::Serialize, Debug)]
pub struct ControlSatelliteRequest {
    pub vel_x: I32F32,
    pub vel_y: I32F32,
    pub camera_angle: &'static str,
    pub state: &'static str,
}

impl JSONBodyHTTPRequestType for ControlSatelliteRequest {
    type Body = ControlSatelliteRequest;
    fn body(&self) -> &Self::Body { self }
}

impl HTTPRequestType for ControlSatelliteRequest {
    type Response = ControlSatelliteResponse;
    fn endpoint(&self) -> &'static str { "/control" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
}
