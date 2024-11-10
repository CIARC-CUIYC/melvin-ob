use crate::http_handler::http_handler_common::{ZonedObjective, BeaconObjective};
use super::request_common::HTTPRequestType;
use super::modify_objective::ModifyObjectiveResponse;

#[derive(serde::Serialize)]
struct ModifyObjectiveRequest {
    zoned_objectives: Vec<ZonedObjective>,
    beacon_objectives: Vec<BeaconObjective>
}

impl HTTPRequestType for ModifyObjectiveRequest {
    type Response = ModifyObjectiveResponse;
    type Body = ModifyObjectiveRequest;
    fn endpoint(&self) -> &str { "/objective" }
    fn body(&self) -> &Self::Body {self}
}