use crate::flight_control::objective::{
    beacon_objective::BeaconObjective, known_img_objective::KnownImgObjective,
    secret_img_objective::SecretImgObjective,
};

pub enum ObjectiveType {
    BeaconObj(BeaconObjective),
    KnownImgObj(KnownImgObjective),
    SecretImgObj(SecretImgObjective),
}
