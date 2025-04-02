use crate::imaging::CameraAngle;
use crate::util::Vec2D;
use crate::http_handler::{ImageObjective, ZoneType};
use chrono::{DateTime, Utc};
use fixed::types::I32F32;
use num::ToPrimitive;
use std::cmp::Ordering;

/// Represents a known image objective that specifies a region of interest on the map.
///
/// This objective includes details like the time frame, required camera angle, and coverage percentage.
#[derive(Debug, Clone)]
pub struct KnownImgObjective {
    /// Unique identifier for the objective.
    id: usize,
    /// Human-readable name for the objective.
    name: String,
    /// Start time of the objective in UTC.
    start: DateTime<Utc>,
    /// End time of the objective in UTC.
    end: DateTime<Utc>,
    /// Coordinates of the objective zone as `[x_min, y_min, x_max, y_max]`.
    zone: [i32; 4],
    /// Required camera angle for the objective.
    optic_required: CameraAngle,
    /// Coverage percentage required for the objective.
    coverage_required: f64,
}

impl KnownImgObjective {
    /// Constructs a new [`KnownImgObjective`] from the provided parameters.
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

    /// Returns the unique identifier of the objective.
    pub fn id(&self) -> usize { self.id }
    /// Returns the start time of the objective.
    pub fn start(&self) -> DateTime<Utc> { self.start }
    /// Returns the end time of the objective.
    pub fn end(&self) -> DateTime<Utc> { self.end }
    /// Returns the human-readable name of the objective.
    pub fn name(&self) -> &str { &self.name }
    /// Returns the zone coordinates of the objective.
    pub fn zone(&self) -> [i32; 4] { self.zone }
    /// Returns the required camera angle for the objective.
    pub fn optic_required(&self) -> CameraAngle { self.optic_required }
    /// Returns the coverage percentage required for the objective.
    pub fn coverage_required(&self) -> f64 { self.coverage_required }
    /// Returns the width of the zone.
    pub fn width(&self) -> i32 { self.zone[2] - self.zone[0] }
    /// Returns the height of the zone.
    pub fn height(&self) -> i32 { self.zone[3] - self.zone[1] }

    /// Calculates the central point of the image zone and wraps it around the map if necessary.
    pub fn get_single_image_point(&self) -> Vec2D<I32F32> {
        let x_size = self.zone[2] - self.zone[0];
        let y_size = self.zone[3] - self.zone[1];
        let pos = Vec2D::new(self.zone[0] + x_size / 2, self.zone[1] + y_size / 2);
        Vec2D::new(I32F32::from(pos.x()), I32F32::from(pos.y())).wrap_around_map()
    }

    /// Returns the corners of the zone as pairs of points with their opposite corners.
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

    /// Calculates the minimum number of images needed to meet the coverage requirements.
    ///
    /// # Returns
    /// The minimum number of images as an integer.
    #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
    pub fn min_images(&self) -> i32 {
        let lens_square_side_length = u32::from(self.optic_required().get_square_side_length());
        let zone_width = self.zone[2] - self.zone[0];
        let zone_height = self.zone[3] - self.zone[1];

        let total_zone_area_size = f64::from(zone_width * zone_height);
        let lens_area_size = f64::from(lens_square_side_length.pow(2));
        let min_area_required = total_zone_area_size * self.coverage_required;

        let min_number_of_images_required = (min_area_required / lens_area_size).ceil();
        min_number_of_images_required.to_i32().unwrap()
    }
}

impl TryFrom<ImageObjective> for KnownImgObjective {
    type Error = std::io::Error;

    /// Attempts to convert an [`ImageObjective`] into a [`KnownImgObjective`].
    ///
    /// # Errors
    /// Returns an error if the provided [`ImageObjective`] is of type `SecretZone`.
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

impl TryFrom<(ImageObjective, [i32; 4])> for KnownImgObjective {
    type Error = std::io::Error;

    /// Attempts to convert a tuple of `(ImageObjective, zone)` into a `KnownImgObjective`.
    ///
    /// # Errors
    /// Returns an error if the `ImageObjective` is of type `KnownZone`.
    fn try_from(obj_with_zone: (ImageObjective, [i32; 4])) -> Result<Self, Self::Error> {
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
    /// Compares the equality of two `KnownImgObjective` instances based on their end time.
    fn eq(&self, other: &Self) -> bool { self.end == other.end }
}

impl PartialOrd<Self> for KnownImgObjective {
    /// Partially compares two `KnownImgObjective` instances based on their end time.
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

impl Ord for KnownImgObjective {
    /// Compares two `KnownImgObjective` instances based on their end time.
    fn cmp(&self, other: &Self) -> Ordering { self.end.cmp(&other.end) }
}
