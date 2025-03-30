use crate::http_handler::{
    common::{BeaconObjective, ImageObjective},
    http_response::response_common::SerdeJSONBodyHTTPResponseType,
};

/// Response type for the /objective endpoint -> GET
#[derive(serde::Deserialize, Debug)]
pub(crate) struct ObjectiveListResponse {
    /// `Vec` of zoned objectives, or here `ImageObjectives`
    #[serde(rename = "zoned_objectives")]
    img_objectives: Vec<ImageObjective>,
    /// `Vec` of `BeaconObjectives`
    beacon_objectives: Vec<BeaconObjective>,
}

impl SerdeJSONBodyHTTPResponseType for ObjectiveListResponse {}

impl ObjectiveListResponse {
    /// Returns the list of imaging objectives
    pub(crate) fn img_objectives(&self) -> &Vec<ImageObjective> { &self.img_objectives }
    /// Returns the list of beacon objectives
    pub(crate) fn beacon_objectives(&self) -> &Vec<BeaconObjective> { &self.beacon_objectives }
}
