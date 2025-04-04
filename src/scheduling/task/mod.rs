//! This module defines various task types and their implementations, 
//! including tasks for image capturing, state switching, and velocity changes.

mod base_task;
mod image_task;
mod switch_state_task;
mod vel_change_task;

pub use switch_state_task::SwitchStateTask;
pub use base_task::Task;
pub use base_task::BaseTask;
pub use image_task::ImageTaskStatus;
