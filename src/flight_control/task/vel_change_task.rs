use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::vec2d::Vec2D;

#[derive(Debug)]
pub struct VelocityChangeTask {
    /// The target state to switch to.
    vel_change: VelocityChangeType,
}

#[derive(Debug)]
pub enum VelocityChangeType {
    AtomicVelChange(Vec2D<f32>),
    SequentialVelChange(Box<[Vec2D<f32>]>),
}

impl VelocityChangeTask {
    pub fn new(vel_change: VelocityChangeType) -> Self { Self { vel_change } }

    pub fn vel_change(&self) -> &VelocityChangeType { &self.vel_change }
}
