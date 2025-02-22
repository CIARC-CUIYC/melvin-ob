use crate::flight_control::{camera_state::CameraAngle, orbit::orbit_base::OrbitBase};
use bitvec::{
    bitbox,
    order::Lsb0,
    prelude::{BitBox, BitRef},
};
use fixed::types::I32F32;
use strum_macros::Display;

/// Represents a closed orbit with a fixed period, image time information, and completion status.
pub struct ClosedOrbit {
    /// The base configuration and parameters of the orbit.
    base_orbit: OrbitBase,
    /// The period of the orbit defined as a tuple:
    /// - First element represents the total orbit time.
    /// - Second and third element represent the x/y-period respectively.
    period: (I32F32, I32F32, I32F32),
    /// Maximum time interval between images that ensures proper coverage of the orbit.
    max_image_dt: I32F32,
    /// A bitvector indicating the completion status of orbit segments.
    done: BitBox<usize, Lsb0>,
}

/// Represents possible errors that can occur when creating or verifying an orbit.
#[derive(Debug, Display)]
pub enum OrbitUsabilityError {
    /// Indicates that the orbit is not closed (i.e., does not have a finite period).
    OrbitNotClosed,
    /// Indicates that the orbit does not have sufficient overlap to image properly.
    OrbitNotEnoughOverlap,
}

impl ClosedOrbit {
    /// Creates a new `ClosedOrbit` instance using a given `OrbitBase` and `CameraAngle`.
    ///
    /// # Arguments
    /// - `try_orbit`: The base orbit to initialize the closed orbit.
    /// - `lens`: The camera lens angle used to determine image overlaps.
    ///
    /// # Returns
    /// - `Ok(ClosedOrbit)` if the orbit is closed and sufficient overlap exists.
    /// - `Err(OrbitUsabilityError)` if the orbit doesn't meet the requirements.
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
                    done: bitbox![usize, Lsb0; 0; period.0.to_num::<usize>()],
                }),
            },
        }
    }

    /// Returns an iterator that reorders the `done` bitvector sequence based on a specified shift.
    ///
    /// # Arguments
    /// - `shift_start`: The starting index of the reordering shift.
    /// - `shift_end`: The ending index of the reordering shift.
    ///
    /// # Returns
    /// - An iterator over the reordered `done` bitvector segment.
    ///
    /// # Panics
    /// - If `shift_start` or `shift_end` exceed the length of the bitvector.
    pub fn get_p_t_reordered(
        &self,
        shift_start: usize,
        shift_end: usize,
    ) -> Box<dyn Iterator<Item=BitRef> + '_> {
        assert!(
            shift_start < self.done.len() && shift_end <= self.done.len(),
            "[FATAL] Shift is larger than the orbit length"
        );
        Box::new(self.done[shift_start..].iter().chain(self.done[..shift_start].iter()).rev().skip(shift_end))
    }

    /// Marks a specified range of orbit segments as completed in the `done` bitvector.
    ///
    /// # Arguments
    /// - `first_i`: The first index of the range to mark as completed.
    /// - `last_i`: The last index of the range to mark as completed.
    pub fn mark_done(&mut self, first_i: usize, last_i: usize) {
        self.done
            .as_mut_bitslice()
            .get_mut(first_i..=last_i)
            .unwrap()
            .iter_mut()
            .for_each(|mut b| *b = true);
    }

    /// Returns the maximum image time interval for the orbit.
    ///
    /// # Returns
    /// - `I32F32` representing the maximum imaging time interval.
    pub fn max_image_dt(&self) -> I32F32 { self.max_image_dt }

    /// Returns a reference to the base orbit configuration.
    ///
    /// # Returns
    /// - A reference to the associated `OrbitBase`.
    pub fn base_orbit_ref(&self) -> &OrbitBase { &self.base_orbit }

    /// Returns the period tuple of the closed orbit.
    ///
    /// # Returns
    /// - A tuple `(I32F32, I32F32, I32F32)` representing the orbit's period.
    pub fn period(&self) -> (I32F32, I32F32, I32F32) { self.period }
}
