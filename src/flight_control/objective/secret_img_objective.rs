use crate::flight_control::camera_state::CameraAngle;
use crate::http_handler::ImageObjective;

#[derive(Debug, Clone)]
pub struct SecretImgObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    optic_required: CameraAngle,
    coverage_required: f32,
}

impl From<ImageObjective> for SecretImgObjective {
    fn from(obj: ImageObjective) -> Self {
        Self {
            id: obj.id(),
            name: String::from(obj.name()),
            start: obj.start(),
            end: obj.end(),
            optic_required: CameraAngle::from(obj.optic_required()),
            coverage_required: obj.coverage_required(),
        }
    }
}
