use crate::flight_control::{
    camera_state::CameraAngle, orbit::orbit_base::OrbitBase,
};
use bitvec::{bitbox, order::Lsb0, prelude::{BitBox, BitRef}};
use fixed::types::{I32F32, I64F64};
use num::ToPrimitive;
use strum_macros::Display;

pub struct ClosedOrbit {
    base_orbit: OrbitBase,
    period: (I32F32, I32F32, I32F32),
    max_image_dt: I32F32,
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
                    done: bitbox![usize, Lsb0; 0; period.0.to_usize().unwrap()],
                }),
            },
        }
    }

    pub fn get_p_t_reordered(
        &self,
        shift_start: usize,
        shift_end: usize,
    ) -> impl Iterator<Item = BitRef> {
        assert!(
            shift_start < self.done.len() && shift_end <= self.done.len(),
            "[FATAL] Shift is larger than the orbit length"
        );
        self.done[shift_start..].iter().chain(self.done[..shift_start].iter()).rev()
            .skip(shift_end)
    }

    pub fn mark_done(&mut self, first_i: usize, last_i: usize) {
        self.done
            .as_mut_bitslice()
            .get_mut(first_i..=last_i)
            .unwrap()
            .iter_mut()
            .for_each(|mut b| *b = true);
    }

    pub fn max_image_dt(&self) -> I32F32 { self.max_image_dt }

    pub fn base_orbit_ref(&self) -> &OrbitBase { &self.base_orbit }

    pub fn period(&self) -> (I32F32, I32F32, I32F32) { self.period }
}
