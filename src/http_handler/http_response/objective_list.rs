use crate::http_handler::common::{BeaconObjective, ZonedObjective};
use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

#[derive(serde::Deserialize, Debug)]
pub struct ObjectiveListResponse {
    zoned_objectives: Vec<ZonedObjective>,
    beacon_objectives: Vec<BeaconObjective>,
}

impl SerdeJSONBodyHTTPResponseType for ObjectiveListResponse {}
