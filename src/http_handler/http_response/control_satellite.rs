use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

#[derive(serde::Deserialize, Debug)]
pub struct ControlSatelliteResponse {
    vel_x: f64,
    vel_y: f64,
    camera_angle: String,
    state: String,
    status: String,
}

impl SerdeJSONBodyHTTPResponseType for ControlSatelliteResponse {}
