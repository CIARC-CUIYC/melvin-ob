use crate::http_handler::{
    common::{BeaconObjective, ImageObjective},
    http_response::response_common::SerdeJSONBodyHTTPResponseType,
};

#[derive(serde::Deserialize, Debug)]
pub struct ObjectiveListResponse {
    zoned_objectives: Vec<ImageObjective>,
    beacon_objectives: Vec<BeaconObjective>,
}

impl SerdeJSONBodyHTTPResponseType for ObjectiveListResponse {}
