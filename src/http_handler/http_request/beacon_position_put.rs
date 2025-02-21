use super::beacon_position::BeaconPositionResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use std::collections::HashMap;

#[derive(Debug)]
pub struct BeaconPositionRequest {
    pub beacon_id: u16,
    pub height: u32,
    pub width: u32,
}

impl NoBodyHTTPRequestType for BeaconPositionRequest {}

impl HTTPRequestType for BeaconPositionRequest {
    type Response = BeaconPositionResponse;
    fn endpoint(&self) -> &'static str { "/beacon" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert("beacon_id", self.beacon_id.to_string());
        query.insert("height", self.height.to_string());
        query.insert("width", self.width.to_string());
        query
    }
}
