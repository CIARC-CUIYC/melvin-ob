use super::configure_simulation;
use super::request_common::{
    HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType, bool_to_string,
};
use std::collections::HashMap;

/// Request type for the /simulation endpoint.
#[derive(Debug)]
#[cfg(debug_assertions)]
pub(crate) struct ConfigureSimulationRequest {
    /// Switch to toggle network simulation on and off.
    pub(crate) is_network_simulation: bool,
    /// The desired timestep multiplier for the simulation backend.
    pub(crate) user_speed_multiplier: u32,
}

impl NoBodyHTTPRequestType for ConfigureSimulationRequest {}

impl HTTPRequestType for ConfigureSimulationRequest {
    /// Type of the expected response.
    type Response = configure_simulation::ConfigureSimulationResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/simulation" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
    /// `HashMap` containing the query param key value pairs.
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert(
            "is_network_simulation",
            bool_to_string(self.is_network_simulation).to_string(),
        );
        query.insert(
            "user_speed_multiplier",
            self.user_speed_multiplier.to_string(),
        );
        query
    }
}
