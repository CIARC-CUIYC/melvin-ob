use std::fmt::{Display, Formatter};
use crate::flight_control::camera_state::CameraAngle;
use super::image_task::ImageTask;
use super::switch_state_task::SwitchStateTask;
use crate::flight_control::common::pinned_dt::PinnedTimeDelay;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::flight_state::FlightState;

#[derive(Debug, Copy, Clone)]
pub struct Task {
    task_type: BaseTask,
    dt: PinnedTimeDelay,
}

#[derive(Debug, Copy, Clone)]
enum BaseTask {
    TASKImageTask(ImageTask),
    TASKSwitchState(SwitchStateTask),
}

impl Display for Task {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let task_type_str = match &self.task_type {
            BaseTask::TASKImageTask(_) => "Image Task",
            BaseTask::TASKSwitchState(task) => &*format!("Switch to {}", task.target_state())
        };
        let end = self.dt.get_end().format("%d %H:%M:%S").to_string();
        write!(f, "Due: {end}, Task: {task_type_str}")

    }
}

impl Task {
    pub fn switch_target(target_state: FlightState, dt: PinnedTimeDelay) -> Self {
        Self {
            task_type: BaseTask::TASKSwitchState(
                SwitchStateTask::new(target_state)
                    .expect("[FATAL] Tried to schedule invalid state switch"),
            ),
            dt,
        }
    }

    pub fn image_task(planned_pos: Vec2D<u32>, lens: CameraAngle, dt: PinnedTimeDelay) -> Self {
        Self{
            task_type: BaseTask::TASKImageTask(
                ImageTask::new(
                    planned_pos,
                    lens
                )
            ),
            dt
        }
    }

    /// Returns a mutable reference to the task's time delay.
    ///
    /// # Returns
    /// - A mutable reference to `PinnedTimeDelay`.
    pub fn dt_mut(&mut self) -> &mut PinnedTimeDelay { &mut self.dt }

    /// Returns an immutable reference to the task's time delay.
    ///
    /// # Returns
    /// - An immutable reference to `PinnedTimeDelay`.
    pub fn dt(&self) -> &PinnedTimeDelay { &self.dt }
}
