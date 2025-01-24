use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

// TODO: 422 Response Code: Validation Error -> not implemented
#[derive(serde::Deserialize, Debug)]
#[cfg(debug_assertions)]
pub struct ConfigureSimulationResponse {
    is_network_simulation: bool,
    user_speed_multiplier: u16
}

impl SerdeJSONBodyHTTPResponseType for ConfigureSimulationResponse {}

