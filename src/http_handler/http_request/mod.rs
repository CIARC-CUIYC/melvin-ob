//! This module provides request and implicit response handling for various HTTP endpoints
//! related to simulation and satellite control. Each submodule corresponds to
//! a specific endpoint, managing its request and response types along with
//! associated logic.
use super::http_response::{
    achievements, annoucements, available_slots, beacon_position, configure_simulation,
    control_satellite, create_backup, daily_map, delete_objective, modify_objective, modify_slot,
    objective_image, objective_list, observation, reset, response_common, restore_backup,
    shoot_image,
};

mod achievements_get;
pub(crate) mod announcements_get;
mod available_slots_get;
pub(crate) mod beacon_position_put;
mod configure_simulation_put;
pub(crate) mod control_put;
mod create_backup_get;
pub(crate) mod daily_map_post;
mod delete_objective_delete;
mod modify_objective_put;
mod modify_slot_put;
pub(crate) mod objective_image_post;
pub(crate) mod objective_list_get;
pub(crate) mod observation_get;
pub(crate) mod request_common;
pub(crate) mod reset_get;
mod restore_backup_put;
pub(crate) mod shoot_image_get;
