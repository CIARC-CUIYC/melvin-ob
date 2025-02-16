use crate::http_handler::{
    common::{BeaconObjective, ImageObjective},
    http_response::response_common::SerdeJSONBodyHTTPResponseType,
};

#[derive(serde::Deserialize, Debug)]
pub struct ObjectiveListResponse {
    #[serde(rename = "zoned_objectives")]
    img_objectives: Vec<ImageObjective>,
    beacon_objectives: Vec<BeaconObjective>,
}

impl SerdeJSONBodyHTTPResponseType for ObjectiveListResponse {}

impl ObjectiveListResponse {
    pub fn img_objectives(&self) -> &Vec<ImageObjective> { &self.img_objectives }
    pub fn beacon_objectives(&self) -> &Vec<BeaconObjective> { &self.beacon_objectives }
}
