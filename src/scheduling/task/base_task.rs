use super::{
    image_task::ImageTask,
    switch_state_task::SwitchStateTask,
    vel_change_task::VelocityChangeTask,
};
use crate::fatal;
use crate::imaging::CameraAngle;
use crate::util::Vec2D;
use crate::flight_control::{FlightState, orbit::BurnSequence};
use chrono::{DateTime, Utc};
use std::fmt::{Display, Formatter};
use strum_macros::Display;

/// Represents a task with a specific type and associated time delay.
/// Tasks can include image capture, state switching, or velocity changes.
#[derive(Debug)]
pub struct Task {
    /// The specific type of the task.
    task_type: BaseTask,
    /// The pinned time delay associated with the task's execution.
    t: DateTime<Utc>,
}

/// An enumeration representing different types of tasks.
///
/// It includes tasks for image capturing ([`TakeImage`]),
/// switching flight states (`SwitchState`), and velocity changes ([`ChangeVelocity`]).
#[derive(Display, Debug)]
pub enum BaseTask {
    /// Task to capture an image.
    TakeImage(ImageTask),
    /// Task to switch to a different flight state.
    SwitchState(SwitchStateTask),
    /// Task to change the velocity, represented by a burn sequence.
    ChangeVelocity(VelocityChangeTask),
}

impl Display for Task {
    /// Formats the task for display purposes.
    ///
    /// The formatted output includes the due time and the task's type.
    /// For some task types, additional details are provided based on the task data.
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let task_type_str = match &self.task_type {
            BaseTask::TakeImage(_) => "Image Task",
            BaseTask::SwitchState(task) => &*format!("Switch to {}", task.target_state()),
            BaseTask::ChangeVelocity(task) => {
                let res_vel = task.burn().sequence_vel().last().unwrap();
                let res_pos = task.burn().sequence_pos().last().unwrap();
                let angle_dev = task.burn().rem_angle_dev();
                &*format!(
                    "Burn to velocity {res_vel} at pos {res_pos}, \
                angle deviation will be {angle_dev}",
                )
            }
        };
        let end = self.t.format("%d %H:%M:%S").to_string();
        write!(f, "Due: {end}, Task: {task_type_str}")
    }
}

impl Task {
    /// Creates a new task for switching to a target flight state.
    ///
    /// # Arguments
    /// - `target_state`: The desired flight state to switch to.
    /// - `t`: The time delay associated with the task's execution.
    ///
    /// # Returns
    /// - A new `Task` instance representing the state switch task.
    ///
    /// # Panics
    /// Panics if the provided `target_state` is invalid for switching.
    pub fn switch_target(target_state: FlightState, t: DateTime<Utc>) -> Self {
        Self {
            task_type: BaseTask::SwitchState(
                SwitchStateTask::new(target_state)
                    .unwrap_or_else(|| fatal!("Tried to schedule invalid state switch")),
            ),
            t,
        }
    }

    /// Creates a new task for image capture.
    ///
    /// # Arguments
    /// - `planned_pos`: The target position for capturing the image.
    /// - `lens`: The camera lens configuration.
    /// - `t`: The time delay associated with the task's execution.
    ///
    /// # Returns
    /// - A new `Task` instance representing the image capture task.
    pub fn image_task(planned_pos: Vec2D<u32>, lens: CameraAngle, t: DateTime<Utc>) -> Self {
        Self { task_type: BaseTask::TakeImage(ImageTask::new(planned_pos, lens)), t }
    }

    /// Creates a new task for velocity change.
    ///
    /// # Arguments
    /// - `burn`: The burn sequence for orbital adjustments.
    /// - `t`: The time delay associated with the task's execution.
    ///
    /// # Returns
    /// - A new `Task` instance representing the velocity change task.
    pub fn vel_change_task(
        burn: BurnSequence,
        t: DateTime<Utc>,
    ) -> Self {
        Self { task_type: BaseTask::ChangeVelocity(VelocityChangeTask::new(burn)), t }
    }
    /// Returns an immutable reference to the task's time delay.
    ///
    /// # Returns
    /// - An `DateTime<Utc>` representing the tasks due time.
    pub fn t(&self) -> DateTime<Utc> { self.t }

    /// Returns an immutable reference to the task's type.
    ///
    /// # Returns
    /// - An immutable reference to the `BaseTask`.
    pub fn task_type(&self) -> &BaseTask { &self.task_type }
}
