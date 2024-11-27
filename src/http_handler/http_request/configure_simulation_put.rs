use super::request_common::{bool_to_header_value, HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::configure_simulation;

#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct ConfigureSimulationRequest {
    pub is_network_simulation: bool,
    pub user_speed_multiplier: u32,
}

impl NoBodyHTTPRequestType for ConfigureSimulationRequest {}

impl HTTPRequestType for ConfigureSimulationRequest {
    type Response = configure_simulation::ConfigureSimulationResponse;
    fn endpoint(&self) -> &str { "/simulation" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.append(
            "is_network_simulation",
            bool_to_header_value(self.is_network_simulation));
        headers.append(
            "user_speed_multiplier",
            reqwest::header::HeaderValue::from(self.user_speed_multiplier),
        );
        headers
    }
}