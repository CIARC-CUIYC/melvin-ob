use super::beacon_position::BeaconPositionResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use std::collections::HashMap;

/// Request type for the /beacon endpoint -> PUT.
#[derive(Debug)]
pub(crate) struct BeaconPositionRequest {
    /// The id of the associated beacon objective.
    pub(crate) beacon_id: u16,
    /// The y-coordinate of the guess.
    pub(crate) height: u32,
    /// The x-coordinate of the guess.
    pub(crate) width: u32,
}

impl NoBodyHTTPRequestType for BeaconPositionRequest {}

impl HTTPRequestType for BeaconPositionRequest {
    /// Type of the expected response.
    type Response = BeaconPositionResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/beacon" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
    /// A `HasMap` containing the query param key value pairs.
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert("beacon_id", self.beacon_id.to_string());
        query.insert("height", self.height.to_string());
        query.insert("width", self.width.to_string());
        query
    }
}
