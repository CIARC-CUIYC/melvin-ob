use crate::http_handler::http_handler_common::{BeaconObjective, ZonedObjective};
use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError, SerdeJSONBodyHTTPResponseType,
};

#[derive(serde::Deserialize, Debug)]
pub struct ObjectiveListResponse {
    zoned_objectives: Vec<ZonedObjective>,
    beacon_objectives: Vec<BeaconObjective>,
}

impl SerdeJSONBodyHTTPResponseType for ObjectiveListResponse {}

impl ObjectiveListResponse {
    pub fn zoned_objectives(&self) -> &Vec<ZonedObjective> {
        &self.zoned_objectives
    }
    pub fn beacon_objectives(&self) -> &Vec<BeaconObjective> {
        &self.beacon_objectives
    }
}
