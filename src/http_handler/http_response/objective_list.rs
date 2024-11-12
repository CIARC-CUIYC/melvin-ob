use crate::http_handler::http_handler_common::{ZonedObjective, BeaconObjective};

#[derive(serde::Deserialize, Debug)]
pub struct ObjectiveListResponse{
    zoned_objectives: Vec<ZonedObjective>,
    beacon_objectives: Vec<BeaconObjective>
}