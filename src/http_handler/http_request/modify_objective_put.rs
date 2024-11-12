use crate::http_handler::http_handler_common::{ZonedObjective, BeaconObjective};
use super::request_common::{HTTPRequest, HTTPRequestType};
use super::modify_objective::ModifyObjectiveResponse;

#[cfg(debug_assertions)]
#[derive(serde::Serialize, Debug)]
pub struct ModifyObjectiveRequest {
    pub zoned_objectives: Vec<ZonedObjective>,
    pub beacon_objectives: Vec<BeaconObjective>,
}

impl Into<HTTPRequest<Self>> for ModifyObjectiveRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Put(self)
    }
}

impl HTTPRequestType for ModifyObjectiveRequest {
    type Response = ModifyObjectiveResponse;
    type Body = ModifyObjectiveRequest;
    fn endpoint(&self) -> &str { "/objective" }
    fn body(&self) -> &Self::Body { self }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.insert("Content-Type",
                       reqwest::header::HeaderValue::from_static("application/json"));
        headers
    }
}