use crate::flight_control::camera_state::CameraAngle;
use strum_macros::Display;

#[derive(Display, Debug, Clone)]
pub enum ObjectiveType {
    Beacon {
        attempts_made: u32,
    },
    KnownImage {
        zone: [i32; 4],
        optic_required: CameraAngle,
        coverage_required: f64,
    },
}
