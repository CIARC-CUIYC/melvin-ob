use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::objective::objective_type::ObjectiveType;
use crate::http_handler::{BeaconObjective, ImageObjective, ZoneType};

#[derive(Debug, Clone)]
pub struct ObjectiveBase {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    obj_type: ObjectiveType,
}

impl ObjectiveBase {
    pub fn id(&self) -> usize { self.id }
    pub fn name(&self) -> &str { &self.name }
    pub fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    pub fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
    pub fn obj_type(&self) -> &ObjectiveType { &self.obj_type }
}

impl From<ImageObjective> for ObjectiveBase {
    fn from(obj: ImageObjective) -> Self {
        let obj_type = match obj.zone_type() {
            ZoneType::KnownZone(zone) => ObjectiveType::KnownImage {
                zone: *zone,
                optic_required: CameraAngle::from(obj.optic_required()),
                coverage_required: obj.coverage_required(),
            },
            ZoneType::SecretZone(_) => ObjectiveType::SecretImage {
                optic_required: CameraAngle::from(obj.optic_required()),
                coverage_required: obj.coverage_required(),
            },
        };
        Self {
            id: obj.id(),
            name: String::from(obj.name()),
            start: obj.start(),
            end: obj.end(),
            obj_type,
        }
    }
}

impl From<BeaconObjective> for ObjectiveBase {
    fn from(obj: BeaconObjective) -> Self {
        let obj_type = ObjectiveType::Beacon;
        Self {
            id: obj.id(),
            name: String::from(obj.name()),
            start: obj.start(),
            end: obj.end(),
            obj_type,
        }
    }
}
