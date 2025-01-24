use super::configure_simulation;
use super::request_common::{
    bool_to_string, HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType,
};
use std::collections::HashMap;

#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct ConfigureSimulationRequest {
    pub is_network_simulation: bool,
    pub user_speed_multiplier: u32,
}

impl NoBodyHTTPRequestType for ConfigureSimulationRequest {}

impl HTTPRequestType for ConfigureSimulationRequest {
    type Response = configure_simulation::ConfigureSimulationResponse;
    fn endpoint(&self) -> &'static str { "/simulation" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
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
