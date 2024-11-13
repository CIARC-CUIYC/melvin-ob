use crate::http_handler::http_response::response_common::{HTTPResponseType,
                                                          JSONBodyHTTPResponseType, ResponseError};

// TODO: 422 Response Code: Validation Error -> not implemented

#[derive(serde::Deserialize, Debug)]
pub struct ControlSatelliteResponse {
    vel_x: f64,
    vel_y: f64,
    camera_angle: String,
    state: String,
    status: String,
}

impl JSONBodyHTTPResponseType for ControlSatelliteResponse {}

impl HTTPResponseType for ControlSatelliteResponse {
    type ParsedResponseType = Self;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response)?;
        Ok(Self::parse_json_body(response).await?)
    }
}
