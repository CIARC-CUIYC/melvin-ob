use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::orbit::burn_sequence::BurnSequence;

#[derive(Debug)]
pub struct VelocityChangeTask {
    burn: BurnSequence,
}

impl VelocityChangeTask {
    pub fn new(burn: BurnSequence) -> Self { Self { burn } }

    pub fn vel_change(&self) -> &BurnSequence { &self.burn }
}
