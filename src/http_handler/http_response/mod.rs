//! This module defines and organizes all the submodules for handling various 
//! HTTP endpoints and their corresponding responses. Each submodule represents
//! an implementation related to a specific API endpoint, including its
//! response handling and parsing logic.
pub(super) mod achievements;
pub(crate) mod annoucements;
pub(super) mod available_slots;
pub(crate) mod beacon_position;
pub(super) mod configure_simulation;
pub(crate) mod control_satellite;
pub(super) mod create_backup;
pub(crate) mod daily_map;
pub(super) mod delete_objective;
pub(super) mod modify_objective;
pub(super) mod modify_slot;
pub(crate) mod objective_image;
pub(crate) mod objective_list;
pub(crate) mod observation;
pub(crate) mod reset;
pub(super) mod response_common;
pub(super) mod restore_backup;
pub(crate) mod shoot_image;
