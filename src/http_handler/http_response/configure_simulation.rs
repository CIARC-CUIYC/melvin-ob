use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

/// Response type for the /simulation endpoint
#[derive(serde::Deserialize, Debug)]
#[cfg(debug_assertions)]
pub(crate) struct ConfigureSimulationResponse {
    /// `true` if network simulation mode is enabled
    is_network_simulation: bool,
    /// Simulation Step speed multiplier
    user_speed_multiplier: u16,
}

impl SerdeJSONBodyHTTPResponseType for ConfigureSimulationResponse {}
