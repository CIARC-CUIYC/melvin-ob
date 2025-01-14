use super::control_satellite::ControlSatelliteResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, JSONBodyHTTPRequestType};

#[derive(serde::Serialize, Debug)]
pub struct ControlSatelliteRequest {
    pub vel_x: f32,
    pub vel_y: f32,
    pub camera_angle: &'static str,
    pub state: &'static str,
}

impl JSONBodyHTTPRequestType for ControlSatelliteRequest {
    type Body = ControlSatelliteRequest;
    fn body(&self) -> &Self::Body {
        self
    }
}

impl HTTPRequestType for ControlSatelliteRequest {
    type Response = ControlSatelliteResponse;
    fn endpoint(&self) -> &str {
        "/control"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Put
    }
}
