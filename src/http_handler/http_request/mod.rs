use super::http_response::*;

pub mod available_slots_get;
pub(super) mod request_common;
pub mod modify_slot_put;
pub mod create_backup_get;
pub mod restore_backup_put;
pub mod objective_list_get;
pub mod modify_objective_put;
pub mod delete_objective_delete;
pub mod configure_simulation_put;
pub mod achievements_get;
mod observation_get;
mod control_put;
//pub(crate) use request_common::{HTTPRequestType, HTTPRequest};