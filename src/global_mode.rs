use crate::flight_control::objective::beacon_objective::BeaconObjective;
use crate::flight_control::objective::known_img_objective::KnownImgObjective;
use strum_macros::Display;

#[derive(Display, Clone)]
pub enum GlobalMode {
    MappingMode,
    ZonedObjectivePrepMode(KnownImgObjective),
    ZonedObjectiveRetrievalMode(KnownImgObjective),
    ZonedObjectiveReturnMode,
    BeaconObjectivePassiveScanningMode(BeaconObjective),
    BeaconObjectiveActiveScanningMode(BeaconObjective),
}

impl GlobalMode {
    pub fn should_map(&self) -> bool {
        match self {
            GlobalMode::MappingMode
            | GlobalMode::BeaconObjectivePassiveScanningMode(_)
            | GlobalMode::ZonedObjectivePrepMode(_) => true,
            _ => false,
        }
    }
}
