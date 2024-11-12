use super::request_common::{HTTPRequest, HTTPRequestType};
use super::control_satellite::ControlSatelliteResponse;

#[cfg(debug_assertions)]
#[derive(serde::Serialize)]
pub struct ControlSatelliteRequest {
    pub vel_x: f64,
    pub vel_y: f64,
    pub camera_angle: String,
    pub state: String
}

impl Into<HTTPRequest<Self>> for ControlSatelliteRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Put(self)
    }
}

impl HTTPRequestType for ControlSatelliteRequest {
    type Response = ControlSatelliteResponse;
    type Body = ControlSatelliteRequest;
    fn endpoint(&self) -> &str { "/simulation" }
    fn body(&self) -> &Self::Body { &self }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.insert("Content-Type",
                       reqwest::header::HeaderValue::from_static("application/json"));
        headers
    }
}