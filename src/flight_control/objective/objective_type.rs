use crate::flight_control::camera_state::CameraAngle;
use strum_macros::Display;

#[derive(Display, Debug, Clone)]
pub enum ObjectiveType {
    Beacon,
    KnownImage {
        zone: [i32; 4],
        optic_required: CameraAngle,
        coverage_required: f32,
    },
    SecretImage {
        optic_required: CameraAngle,
        coverage_required: f32,
    },
}
