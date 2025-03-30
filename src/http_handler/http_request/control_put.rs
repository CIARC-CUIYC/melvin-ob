use super::control_satellite::ControlSatelliteResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, JSONBodyHTTPRequestType};

/// Request type for the /control endpoint.
#[derive(serde::Serialize, Debug)]
pub(crate) struct ControlSatelliteRequest {
    /// The desired velocity in x-direction.
    pub(crate) vel_x: f64,
    /// The desired velocity in y-direction.
    pub(crate) vel_y: f64,
    /// The desired `CameraAngle` encoded as a `str`.
    pub(crate) camera_angle: &'static str,
    /// The desired `FlightState` encoded as a `str`.
    pub(crate) state: &'static str,
}

impl JSONBodyHTTPRequestType for ControlSatelliteRequest {
    /// The type of the json body.
    type Body = ControlSatelliteRequest;
    /// Returns the serializable object.
    fn body(&self) -> &Self::Body { self }
}

impl HTTPRequestType for ControlSatelliteRequest {
    /// Type of the expected response.
    type Response = ControlSatelliteResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/control" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
}
