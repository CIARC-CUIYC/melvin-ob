use super::request_common::{HTTPRequest, HTTPRequestType};
use super::beacon_position::BeaconPositionResponse;

#[derive(serde::Serialize)]
pub struct BeaconPositionRequest {
    pub beacon_id: i8,
    pub height: i16,
    pub width: i16
}

impl Into<HTTPRequest<Self>> for BeaconPositionRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Put(self)
    }
}

impl HTTPRequestType for BeaconPositionRequest {
    type Response = BeaconPositionResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/simulation" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.append("beacon_id", i32::from(self.beacon_id).into());
        headers.append("height", self.height.into());
        headers.append("width", self.width.into());
        headers
    }
}