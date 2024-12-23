use super::modify_objective;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, JSONBodyHTTPRequestType};
use crate::http_handler::http_handler_common::{BeaconObjective, ZonedObjective};

#[derive(serde::Serialize, Debug)]
#[cfg(debug_assertions)]
pub struct ModifyObjectiveRequest {
    pub zoned_objectives: Vec<ZonedObjective>,
    pub beacon_objectives: Vec<BeaconObjective>,
}

impl JSONBodyHTTPRequestType for ModifyObjectiveRequest {
    type Body = Self;
    fn body(&self) -> &Self::Body {
        &self
    }
}

impl HTTPRequestType for ModifyObjectiveRequest {
    type Response = modify_objective::ModifyObjectiveResponse;
    fn endpoint(&self) -> &str {
        "/objective"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Put
    }
}
