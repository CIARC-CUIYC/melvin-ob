mod atomic_decision;
mod atomic_decision_cube;
pub(crate) mod base_task;
pub(crate) mod image_task;
mod score_grid;
pub(crate) mod switch_state_task;
mod task_controller;
pub(crate) mod vel_change_task;

pub use task_controller::TaskController;