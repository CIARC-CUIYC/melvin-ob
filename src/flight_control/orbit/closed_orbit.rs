use bitvec::{bitbox, order::Lsb0, prelude::BitBox};
use bitvec::slice::Iter;
use strum_macros::Display;
use crate::flight_control::{camera_state::CameraAngle,
                            common::vec2d::Vec2D,
                            orbit::orbit_base::OrbitBase};

pub struct ClosedOrbit {
    base_orbit: OrbitBase,
    period: (f64, f64, f64),
    max_image_dt: f32,
    done: BitBox<usize, Lsb0>,
}

#[derive(Debug, Display)]
pub enum OrbitUsabilityError {
    OrbitNotClosed,
    OrbitNotEnoughOverlap,
}

impl ClosedOrbit {

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    pub fn new(try_orbit: OrbitBase, lens: CameraAngle) -> Result<Self, OrbitUsabilityError> {
        match try_orbit.period() {
            None => Err(OrbitUsabilityError::OrbitNotClosed),
            Some(period) => match try_orbit.max_image_dt(lens, period) {
                None => Err(OrbitUsabilityError::OrbitNotEnoughOverlap),
                Some(max_image_dt) => Ok(Self {
                    base_orbit: try_orbit,
                    period,
                    max_image_dt,
                    done: bitbox![usize, Lsb0; 0; period.0 as usize],
                }),
            },
        }
    }

    pub fn get_p_t_reordered(&self, pos: Vec2D<f32>) -> Iter<usize, Lsb0> {
        self.done.iter()
        /*
        match self.base_orbit.will_visit(pos) {
            true => {}
            false => {}
        }*/
    }

    pub fn max_image_dt(&self) -> f32 { self.max_image_dt }

    pub fn base_orbit_ref(&self) -> &OrbitBase { &self.base_orbit }

    pub fn period(&self) -> (f64, f64, f64) { self.period }
}