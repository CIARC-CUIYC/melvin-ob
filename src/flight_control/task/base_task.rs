use super::image_task::ImageTask;
use super::switch_state_task::SwitchStateTask;
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::pinned_dt::PinnedTimeDelay;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::flight_state::FlightState;
use crate::flight_control::task::vel_change_task::{VelocityChangeTask, VelocityChangeType};
use std::fmt::{Display, Formatter};
use strum_macros::Display;

#[derive(Debug)]
pub struct Task {
    task_type: BaseTask,
    dt: PinnedTimeDelay,
}

#[derive(Display, Debug)]
pub enum BaseTask {
    TakeImage(ImageTask),
    SwitchState(SwitchStateTask),
    ChangeVelocity(VelocityChangeTask),
}

impl Display for Task {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let task_type_str = match &self.task_type {
            BaseTask::TakeImage(_) => "Image Task",
            BaseTask::SwitchState(task) => &*format!("Switch to {}", task.target_state()),
            BaseTask::ChangeVelocity(task) => match task.vel_change() {
                VelocityChangeType::AtomicVelChange(ch) => &*format!("Atomic vel change to {ch}."),
                VelocityChangeType::SequentialVelChange(ch) => {
                    &*format!("Atomic vel change to {}.", ch.last().unwrap())
                }
            },
        };
        let end = self.dt.get_end().format("%d %H:%M:%S").to_string();
        write!(f, "Due: {end}, Task: {task_type_str}")
    }
}

impl Task {
    pub fn switch_target(target_state: FlightState, dt: PinnedTimeDelay) -> Self {
        Self {
            task_type: BaseTask::SwitchState(
                SwitchStateTask::new(target_state)
                    .expect("[FATAL] Tried to schedule invalid state switch"),
            ),
            dt,
        }
    }

    pub fn image_task(planned_pos: Vec2D<u32>, lens: CameraAngle, dt: PinnedTimeDelay) -> Self {
        Self {
            task_type: BaseTask::TakeImage(ImageTask::new(planned_pos, lens)),
            dt,
        }
    }

    pub fn vel_change_task(vel_change: VelocityChangeType, dt: PinnedTimeDelay) -> Self {
        Self {
            task_type: BaseTask::ChangeVelocity(VelocityChangeTask::new(vel_change)),
            dt,
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

    pub fn task_type(&self) -> &BaseTask { &self.task_type }
}
