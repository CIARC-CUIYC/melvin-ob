use super::image_task::ImageTask;
use super::switch_state_task::SwitchStateTask;
use super::vel_change_task::VelocityChangeTask;
use crate::flight_control::{
    camera_state::CameraAngle,
    common::{pinned_dt::PinnedTimeDelay, vec2d::Vec2D},
    flight_state::FlightState,
    orbit::burn_sequence::BurnSequence,
};
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
            BaseTask::ChangeVelocity(task) => {
                let res_vel = task.vel_change().sequence_vel().last().unwrap();
                let res_pos = task.vel_change().sequence_pos().last().unwrap();
                let angle_dev = task.vel_change().res_angle_dev();
                &*format!(
                    "Burn to velocity {res_vel} at pos {res_pos}, \
                angle deviation will be {angle_dev}",
                )
            }
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

    pub fn vel_change_task(burn: BurnSequence, dt: PinnedTimeDelay) -> Self {
        Self {
            task_type: BaseTask::ChangeVelocity(VelocityChangeTask::new(burn)),
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
