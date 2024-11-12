use super::request_common::{bool_to_header_value, HTTPRequest, HTTPRequestType};
use super::configure_simulation::ConfigureSimulationResponse;

#[cfg(debug_assertions)]
#[derive(serde::Serialize)]
pub struct ConfigureSimulationRequest {
    pub is_network_simulation: bool,
    pub user_speed_multiplier: u32,
}

impl Into<HTTPRequest<Self>> for ConfigureSimulationRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Put(self)
    }
}

impl HTTPRequestType for ConfigureSimulationRequest {
    type Response = ConfigureSimulationResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/simulation" }
    fn body(&self) -> &Self::Body { &() }
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