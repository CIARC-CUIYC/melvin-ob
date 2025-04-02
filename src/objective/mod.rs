mod beacon_objective;
mod beacon_objective_done;
mod known_img_objective;
mod secret_img_objective;
mod bayesian_set;
mod beacon_controller;

use bayesian_set::BayesianSet;
use beacon_objective::BeaconMeas;

pub use beacon_objective::BeaconObjective;
pub use known_img_objective::KnownImgObjective;
pub use beacon_controller::BeaconController;
pub use beacon_controller::BeaconControllerState;

#[cfg(test)]
mod tests;
