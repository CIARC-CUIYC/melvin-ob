use super::orbit_base::OrbitBase;
use crate::flight_control::{
    camera_state::CameraAngle,
    common::vec2d::{Vec2D, VecAxis},
};
use crate::{fatal, warn};
use bincode::config::{Configuration, Fixint, LittleEndian};
use bincode::error::EncodeError;
use bitvec::{
    bitbox,
    order::Lsb0,
    prelude::{BitBox, BitRef},
};
use fixed::types::I32F32;
use std::env;
use strum_macros::Display;

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub(super) struct OrbitSegment {
    start: Vec2D<I32F32>,
    end: Vec2D<I32F32>,
    delta: Vec2D<I32F32>,
}

impl OrbitSegment {
    fn new(start: Vec2D<I32F32>, end: Vec2D<I32F32>) -> Self {
        let delta = end - start;
        Self { start, end, delta }
    }
    pub(crate) fn start(&self) -> &Vec2D<I32F32> { &self.start }
    pub(crate) fn end(&self) -> &Vec2D<I32F32> { &self.end }

    fn get_proj_dist(&self, pos: &Vec2D<I32F32>) -> (VecAxis, I32F32) {
        let (t_x, t_y) = self.tx_tys(pos);

        if t_x.is_negative() || t_x > I32F32::ONE || t_y.is_negative() || t_y > I32F32::ONE {
            return (VecAxis::X, I32F32::MAX);
        }

        let proj_x = self.start.x() + self.delta.x() * t_y;
        let proj_y = self.start.y() + self.delta.y() * t_x;

        let deviation_x = proj_x - pos.x();
        let deviation_y = proj_y - pos.y();

        if deviation_x.abs() < deviation_y.abs() {
            (VecAxis::X, deviation_x)
        } else {
            (VecAxis::Y, deviation_y)
        }
    }

    fn tx_tys(&self, pos: &Vec2D<I32F32>) -> (I32F32, I32F32) {
        let t_x = if self.delta.x().abs() > I32F32::DELTA {
            (pos.x() - self.start.x()) / self.delta.x()
        } else {
            I32F32::ZERO
        };

        let t_y = if self.delta.y().abs() > I32F32::DELTA {
            (pos.y() - self.start.y()) / self.delta.y()
        } else {
            I32F32::ZERO
        };
        (t_x, t_y)
    }

    fn get_abs_dist(&self, pos: &Vec2D<I32F32>) -> Vec2D<I32F32> {
        let (t_x, t_y) = self.tx_tys(pos);
        let t_x_pos = *self.start() + self.delta * t_x;
        let t_y_pos = *self.start() + self.delta * t_y;
        let midpoint = (t_x_pos + t_y_pos) / I32F32::from_num(2);
        pos.to(&midpoint)
    }
}

/// Represents a closed orbit with a fixed period, image time information, and completion status.
#[derive(serde::Serialize, serde::Deserialize, Debug)]
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

    segments: Vec<OrbitSegment>,
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
    const EXPORT_ORBIT_ENV: &'static str = "EXPORT_ORBIT";
    const TRY_IMPORT_ENV: &'static str = "TRY_IMPORT_ORBIT";
    const DEF_FILEPATH: &'static str = "orbit.bin";
    /// Creates a new [`ClosedOrbit`] instance using a given [`OrbitBase`] and [`CameraAngle`].
    ///
    /// # Arguments
    /// - `try_orbit`: The base orbit to initialize the closed orbit.
    /// - `lens`: The camera lens angle used to determine image overlaps.
    ///
    /// # Returns
    /// - `Ok(ClosedOrbit)` if the orbit is closed and sufficient overlap exists.
    /// - `Err(OrbitUsabilityError)` if the orbit doesn't meet the requirements.
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    pub fn new(base_orbit: OrbitBase, lens: CameraAngle) -> Result<Self, OrbitUsabilityError> {
        match base_orbit.period() {
            None => Err(OrbitUsabilityError::OrbitNotClosed),
            Some(period) => match base_orbit.max_image_dt(lens, period) {
                None => Err(OrbitUsabilityError::OrbitNotEnoughOverlap),
                Some(max_image_dt) => {
                    let segments = Self::compute_segments(base_orbit.fp(), base_orbit.vel());
                    let done = bitbox![usize, Lsb0; 0; period.0.to_num::<usize>()];
                    Ok(Self { base_orbit, period, max_image_dt, done, segments })
                }
            },
        }
    }
    
    pub fn clear_done(&mut self) {
        self.done.fill(false);
    }

    pub fn try_from_env() -> Option<Self> {
        if env::var(Self::TRY_IMPORT_ENV).is_ok_and(|s| s == "1") {
            Self::import_from(Self::DEF_FILEPATH).ok()
        } else {
            None
        }        
    }

    pub fn try_export_default(&self) {
        if env::var(Self::EXPORT_ORBIT_ENV).is_ok_and(|s| s == "1") {
            self.export_to(Self::DEF_FILEPATH).unwrap_or_else(|e| {
                warn!("Failed to export orbit: {}", e);
            });
        }
    }
    
    fn import_from(filename: &'static str) -> Result<Self, std::io::Error> {
        let mut file = std::fs::OpenOptions::new().read(true).open(filename)?;
        bincode::serde::decode_from_std_read(&mut file, Self::get_serde_config()).map_err(|e| {
            fatal!("Failed to import orbit from {}: {}", filename, e);
        })
    }
    
    fn export_to(&self, filename: &'static str) -> Result<(), EncodeError> {
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(filename)
            .unwrap();
        bincode::serde::encode_into_std_write(self, &mut file, Self::get_serde_config())?;
        Ok(())
    }

    fn get_serde_config() -> Configuration<LittleEndian, Fixint> {
        bincode::config::standard().with_little_endian().with_fixed_int_encoding()
    }

    fn compute_segments(base_point: &Vec2D<I32F32>, vel: &Vec2D<I32F32>) -> Vec<OrbitSegment> {
        let mut segments = Vec::new();

        let mut current_point = base_point.project_overboundary_bw(vel);
        let mut visited_points = Vec::new();

        loop {
            let min = visited_points
                .iter()
                .map(|p| (p, current_point.euclid_distance(p)))
                .min_by(|&(_, dist1), &(_, dist2)| dist1.cmp(&dist2))
                .map(|(p, _)| p);

            if let Some(min_point) = min {
                if current_point.euclid_distance(min_point) < 2 * vel.abs() {
                    break;
                }
            } else {
                visited_points.push(current_point);
            }

            let next_point = current_point.project_overboundary_fw(vel);
            segments.push(OrbitSegment::new(current_point, next_point));
            // wrap next_point back onto the plane
            current_point = next_point.wrap_around_map().project_overboundary_bw(vel);
        }
        segments
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
    ) -> Box<dyn Iterator<Item = BitRef> + '_> {
        assert!(
            shift_start < self.done.len() && shift_end <= self.done.len(),
            "[FATAL] Shift is larger than the orbit length"
        );
        Box::new(
            self.done[shift_start..]
                .iter()
                .chain(self.done[..shift_start].iter())
                .rev()
                .skip(shift_end),
        )
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

    pub fn get_closest_deviation(&self, pos: Vec2D<I32F32>) -> (VecAxis, I32F32) {
        self.segments
            .iter()
            .map(|seg| seg.get_proj_dist(&pos))
            .min_by(|a, b| a.1.abs().cmp(&b.1.abs()))
            .unwrap()
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

    /// Checks whether the specified position on the map will be visited during the orbit.
    ///
    /// # Arguments
    /// - `pos`: The position to check.
    ///
    /// # Returns
    /// - `true`: If the position will be visited during the orbit.
    /// - `false`: Otherwise.
    pub fn will_visit(&self, pos: Vec2D<I32F32>) -> bool {
        self.segments
            .iter()
            .map(|seg| seg.get_abs_dist(&pos))
            .min_by(|a, b| a.abs().cmp(&b.abs()))
            .unwrap()
            .abs()
            < I32F32::lit("1.0")
    }

    pub fn get_i(&self, pos: Vec2D<I32F32>) -> Option<usize> {
        if self.will_visit(pos) {
            let step = *self.base_orbit.vel();
            let step_abs = step.abs();
            let mut i_pos = *self.base_orbit.fp();
            for i in 0..self.period.0.to_num::<usize>() {
                let mut dx_abs = i_pos.euclid_distance(&pos);
                if dx_abs < step_abs * 2 {
                    let mut next = (i_pos + step).wrap_around_map();
                    let mut add_i = 0;
                    while next.wrap_around_map().euclid_distance(&pos) < dx_abs {
                        add_i += 1;
                        next = (next + step).wrap_around_map();
                        dx_abs = next.euclid_distance(&pos);
                    }
                    return Some(i + add_i);
                }
                i_pos = (i_pos + step).wrap_around_map();
            }
        }
        None
    }

    pub(super) fn segments(&self) -> &Vec<OrbitSegment> { &self.segments }
    
    pub fn get_coverage(&self) -> I32F32 {
        let zeros = I32F32::from_num(self.done.count_zeros());
        let length = I32F32::from_num(self.done.len());
        zeros / length
    }
}
