use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

/// Response type for the /control endpoint.
#[derive(serde::Deserialize, Debug)]
pub(crate) struct ControlSatelliteResponse {
    /// The velocity in x-direction.
    vel_x: f64,
    /// The velocity in y-direction.
    vel_y: f64,
    /// The currently selected `CameraAngle` as a string (e.g. "narrow" or "wide").
    camera_angle: String,
    /// The current `FlightState` as a string (e.g. "acquisition" or "deployment").
    state: String,
    /// The current status of the satellite
    status: String,
}

impl SerdeJSONBodyHTTPResponseType for ControlSatelliteResponse {}
