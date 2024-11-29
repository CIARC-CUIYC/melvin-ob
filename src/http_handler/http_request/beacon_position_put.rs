use super::beacon_position::BeaconPositionResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

#[derive(Debug)]
pub struct BeaconPositionRequest {
    pub beacon_id: i8,
    pub height: i16,
    pub width: i16,
}

impl NoBodyHTTPRequestType for BeaconPositionRequest {}

impl HTTPRequestType for BeaconPositionRequest {
    type Response = BeaconPositionResponse;
    fn endpoint(&self) -> &str {
        "/beacon"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Put
    }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.append("beacon_id", i32::from(self.beacon_id).into());
        headers.append("height", self.height.into());
        headers.append("width", self.width.into());
        headers
    }
}
