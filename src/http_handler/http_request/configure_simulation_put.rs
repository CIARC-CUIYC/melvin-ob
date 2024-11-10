use super::request_common::HTTPRequestType;
use super::configure_simulation::ConfigureSimulationResponse;

#[cfg(debug_assertions)]
#[derive(serde::Serialize)]
struct ConfigureSimulationRequest {
    is_network_simulation: bool,
    user_speed_multiplier: u32
}

impl HTTPRequestType for ConfigureSimulationRequest {
    type Response = ConfigureSimulationResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/simulation" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}