use crate::flight_control::{camera_state::CameraAngle, common::vec2d::Vec2D};
use crate::http_handler::{ImageObjective, ZoneType};
use chrono::{DateTime, Utc};
use fixed::types::I32F32;
use num::ToPrimitive;
use std::cmp::Ordering;

#[derive(Debug, Clone)]
pub struct KnownImgObjective {
    id: usize,
    name: String,
    start: DateTime<Utc>,
    end: DateTime<Utc>,
    zone: [i32; 4],
    optic_required: CameraAngle,
    coverage_required: f64,
}

impl KnownImgObjective {
    pub fn new(
        id: usize,
        name: String,
        start: DateTime<Utc>,
        end: DateTime<Utc>,
        zone: [i32; 4],
        optic_required: CameraAngle,
        coverage_required: f64,
    ) -> KnownImgObjective {
        KnownImgObjective { id, name, start, end, zone, optic_required, coverage_required }
    }

    pub fn id(&self) -> usize { self.id }
    pub fn end(&self) -> DateTime<Utc> { self.end }
    pub fn name(&self) -> &str { &self.name }
    pub fn zone(&self) -> [i32; 4] { self.zone }
    pub fn optic_required(&self) -> CameraAngle { self.optic_required }
    pub fn coverage_required(&self) -> f64 { self.coverage_required }

    pub fn width(&self) -> i32 { self.zone[2] - self.zone[0] }
    pub fn height(&self) -> i32 { self.zone[3] - self.zone[1] }

    pub fn get_single_image_point(&self) -> Vec2D<I32F32> {
        let x_size = self.zone[2] - self.zone[0];
        let y_size = self.zone[3] - self.zone[1];
        let pos = Vec2D::new(self.zone[0] + x_size / 2, self.zone[1] + y_size / 2);
        Vec2D::new(I32F32::from(pos.x()), I32F32::from(pos.y())).wrap_around_map()
        
    }
    
    pub fn get_corners(&self) -> [(Vec2D<I32F32>, Vec2D<I32F32>); 4] {
        let first = Vec2D::new(I32F32::from(self.zone[0]), I32F32::from(self.zone[1]));
        let second = Vec2D::new(I32F32::from(self.zone[0]), I32F32::from(self.zone[3]));
        let third = Vec2D::new(I32F32::from(self.zone[2]), I32F32::from(self.zone[1]));
        let fourth = Vec2D::new(I32F32::from(self.zone[2]), I32F32::from(self.zone[3]));
        [
            (first, first.unwrapped_to(&fourth)),
            (second, second.unwrapped_to(&third)),
            (third, third.unwrapped_to(&second)),
            (fourth, fourth.unwrapped_to(&first)),
        ]
    }

    #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
    pub fn min_images(&self) -> i32 {
        let lens_square_side_length = u32::from(self.optic_required().get_square_side_length());
        let zone_width = self.zone[2] - self.zone[0];
        let zone_height = self.zone[3] - self.zone[1];

        let total_zone_area_size = f64::from(zone_width * zone_height);
        let lens_area_size = f64::from(lens_square_side_length.pow(2));
        let min_area_required = total_zone_area_size * self.coverage_required;

        let min_number_of_images_required = (min_area_required / lens_area_size).floor();
        min_number_of_images_required.to_i32().unwrap()
    }
}

impl TryFrom<ImageObjective> for KnownImgObjective {
    type Error = std::io::Error;

    fn try_from(obj: ImageObjective) -> Result<Self, Self::Error> {
        match obj.zone_type() {
            ZoneType::KnownZone(zone) => Ok(Self {
                id: obj.id(),
                name: String::from(obj.name()),
                start: obj.start(),
                end: obj.end(),
                zone: *zone,
                optic_required: CameraAngle::from(obj.optic_required()),
                coverage_required: obj.coverage_required(),
            }),
            ZoneType::SecretZone(_) => Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "[FATAL] Wrong objective conversion!",
            )),
        }
    }
}

impl TryFrom<(&ImageObjective, [i32; 4])> for KnownImgObjective {
    type Error = std::io::Error;

    fn try_from(obj_with_zone: (&ImageObjective, [i32; 4])) -> Result<Self, Self::Error> {
        let obj = obj_with_zone.0;
        match obj.zone_type() {
            ZoneType::SecretZone(_) => Ok(Self {
                id: obj.id(),
                name: String::from(obj.name()),
                start: obj.start(),
                end: obj.end(),
                zone: obj_with_zone.1,
                optic_required: CameraAngle::from(obj.optic_required()),
                coverage_required: obj.coverage_required(),
            }),
            ZoneType::KnownZone(_) => Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "[FATAL] Wrong objective conversion!",
            )),
        }
    }
}

impl Eq for KnownImgObjective {}

impl PartialEq<Self> for KnownImgObjective {
    fn eq(&self, other: &Self) -> bool { self.end == other.end }
}

impl PartialOrd<Self> for KnownImgObjective {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

impl Ord for KnownImgObjective {
    fn cmp(&self, other: &Self) -> Ordering { self.end.cmp(&other.end) }
}
