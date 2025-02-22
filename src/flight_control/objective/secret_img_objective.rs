use chrono::{DateTime, Utc};
use crate::flight_control::camera_state::CameraAngle;

#[derive(Debug, Clone)]
pub struct SecretImgObjective {
    id: usize,
    name: String,
    start: DateTime<Utc>,
    end: DateTime<Utc>,
    optic_required: CameraAngle,
    coverage_required: f32,
}

impl SecretImgObjective {
    pub fn new(
        id: usize,
        name: String,
        start: DateTime<Utc>,
        end: DateTime<Utc>,
        optic_required: CameraAngle,
        coverage_required: f32,
    ) -> Self {
        Self { id, name, start, end, optic_required, coverage_required }
    }
}
