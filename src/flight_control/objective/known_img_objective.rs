use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::vec2d::Vec2D;
use fixed::types::I32F32;
use num::ToPrimitive;

#[derive(Debug, Clone)]
pub struct KnownImgObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    zone: [i32; 4],
    optic_required: CameraAngle,
    coverage_required: f32,
}

impl KnownImgObjective {
    pub fn new(
        id: usize,
        start: chrono::DateTime<chrono::Utc>,
        end: chrono::DateTime<chrono::Utc>,
        name: String,
        zone: [i32; 4],
        optic_str: &str,
    ) -> KnownImgObjective {
        let optic_required = CameraAngle::from(optic_str);
        KnownImgObjective {
            id,
            start,
            end,
            name,
            zone,
            optic_required,
            coverage_required: 0.0,
        }
    }

    pub fn id(&self) -> usize { self.id }
    pub fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
    pub fn name(&self) -> &str { &self.name }
    pub fn zone(&self) -> [i32; 4] { self.zone }
    pub fn optic_required(&self) -> CameraAngle { self.optic_required }
    pub fn coverage_required(&self) -> f32 { self.coverage_required }

    pub fn get_imaging_points(&self) -> Vec<Vec2D<I32F32>> {
        // TODO: this has to be adapted for multiple imaging points later
        let x_size = self.zone[2] - self.zone[0];
        let y_size = self.zone[3] - self.zone[1];
        let pos = Vec2D::new(self.zone[0] + x_size / 2, self.zone[1] + y_size / 2);
        let pos_fixed = Vec2D::new(I32F32::from(pos.x()), I32F32::from(pos.y())).wrap_around_map();
        vec![pos_fixed]
    }

    #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
    pub fn min_images(&self) -> i32 {
        let lens_square_side_length =
            self.optic_required().get_square_side_length();

        let zone_width = self.zone[2] - self.zone[0];
        let zone_height = self.zone[3] - self.zone[1];

        let total_zone_area_size = (zone_width * zone_height) as f32;
        let lens_area_size = f32::from(lens_square_side_length.pow(2));

        let min_area_required = total_zone_area_size * self.coverage_required;

        let min_number_of_images_required = (min_area_required / lens_area_size).ceil();
        min_number_of_images_required.to_i32().unwrap()
    }
}
