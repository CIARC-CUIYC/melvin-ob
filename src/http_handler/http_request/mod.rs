use super::http_response::{
    achievements, annoucements, available_slots, beacon_position, configure_simulation,
    control_satellite, create_backup, daily_map, delete_objective, modify_objective, modify_slot,
    objective_image, objective_list, observation, reset, response_common, restore_backup,
    shoot_image,
};

pub mod achievements_get;
pub mod announcements_get;
pub mod available_slots_get;
pub mod beacon_position_put;
pub mod configure_simulation_put;
pub mod control_put;
pub mod create_backup_get;
pub mod daily_map_post;
pub mod delete_objective_delete;
pub mod modify_objective_put;
pub mod modify_slot_put;
pub mod objective_image_post;
pub mod objective_list_get;
pub mod observation_get;
pub mod request_common;
pub mod reset_get;
pub mod restore_backup_put;
pub mod shoot_image_get;
