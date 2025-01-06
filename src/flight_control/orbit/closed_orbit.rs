use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::modular_arithmetics::{fmod_f32, gcd_f32, lcm_f32};
use crate::flight_control::common::vec2d::Vec2D;

pub struct Orbit {
    orbit_start: chrono::DateTime<chrono::Utc>,
    fp: Vec2D<f32>,
    vel: Vec2D<f32>,
}

impl Orbit {
    const SIM_TIMESTEP: f32 = 0.5;

    pub fn new(fp: Vec2D<f32>, vel: Vec2D<f32>) -> Self {
        Self {
            orbit_start: chrono::Utc::now(),
            fp,
            vel,
        }
    }

    pub fn start_timestamp(&self) -> chrono::DateTime<chrono::Utc> { self.orbit_start }

    pub fn period(&self) -> Option<f32> {
        let t_x = Vec2D::<f32>::map_size().x() / gcd_f32(self.vel.x(), Vec2D::map_size().x());
        let t_y = Vec2D::<f32>::map_size().y() / gcd_f32(self.vel.y(), Vec2D::map_size().y());
        let tts = lcm_f32(t_x, t_y);
        let mut resulting_point = self.fp + self.vel * tts;
        resulting_point.wrap_around_map();
        let resulting_dist = resulting_point - self.fp;
        if resulting_dist.x().abs() < f32::EPSILON && resulting_dist.y().abs() < f32::EPSILON {
            Some(tts)
        } else {
            None
        }
    }

    pub fn max_image_dt(&self, used_lens: CameraAngle) -> Option<f32> {
        let img_side_length = used_lens.get_square_side_length() as f32;
        let ver_wrap_hor_dist = Vec2D::<f32>::map_size().y() / self.vel.y() * self.vel.x();
        let hor_wrap_ver_dist = Vec2D::<f32>::map_size().x() / self.vel.x() * self.vel.y();

        let dominant_vel = self.vel.x().max(self.vel.y());
        let overlap_hor = ver_wrap_hor_dist - (img_side_length / 2.0);
        let overlap_ver = hor_wrap_ver_dist - (img_side_length / 2.0);
        if overlap_hor < img_side_length / 2.0 || overlap_ver < img_side_length / 2.0 {
            if self.vel.x() / Vec2D::<f32>::map_size().x()
                < self.vel.y() / Vec2D::<f32>::map_size().y()
            {
                Some((overlap_hor / self.vel.x()).min((img_side_length / 2.0) / dominant_vel))
            } else {
                Some((overlap_ver / self.vel.y()).min((img_side_length / 2.0) / dominant_vel))
            }
        } else {
            None
        }
    }

    pub fn is_closed(&self) -> bool { self.period().is_some() }

    pub fn will_visit(&self, pos: Vec2D<f32>) -> bool {
        let pos_dist = pos - self.fp;
        let map_x = Vec2D::<f32>::map_size().x();
        let map_y = Vec2D::<f32>::map_size().y();
        // Compute displacements modulo the map dimensions
        let delta_x = fmod_f32(pos_dist.x(), map_x);
        let delta_y = fmod_f32(pos_dist.y(), map_y);

        // Check if the displacements align with the velocity ratios
        if self.vel.x().abs() > f32::EPSILON && self.vel.y().abs() > f32::EPSILON {
            let tx = delta_x / self.vel.x();
            let ty = delta_y / self.vel.y();
            // Check if the times align within tolerance
            ((tx % map_x) - (ty % map_y)).abs() < f32::EPSILON
        } else {
            false
        }
    }
}
