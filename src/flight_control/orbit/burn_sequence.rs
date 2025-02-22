use crate::flight_control::common::vec2d::Vec2D;
use super::index::IndexedOrbitPosition;
use fixed::types::I32F32;

/// Represents a sequence of corrective burns for orbital adjustments.
///
/// The `BurnSequence` contains position and velocity sequences, along with
/// timing and cost information, for controlling orbit behavior.
#[derive(Debug, Clone)]
pub struct BurnSequence {
    /// The orbital position where the sequence starts.
    start_i: IndexedOrbitPosition,
    /// The sequence of positional corrections.
    sequence_pos: Box<[Vec2D<I32F32>]>,
    /// The sequence of velocity corrections.
    sequence_vel: Box<[Vec2D<I32F32>]>,
    /// Acceleration time in seconds.
    acc_dt: usize,
    /// Time duration for detumbling after acceleration, in seconds.
    detumble_dt: usize,
    /// Factor representing the cost associated with the burn sequence.
    cost_factor: I32F32,
    /// Residual angular deviation allowed after the sequence.
    res_angle_dev: I32F32,
}

impl BurnSequence {
    /// Creates a new `BurnSequence` with the provided parameters.
    ///
    /// # Parameters
    /// * `start_i` - The initial orbital position for the sequence.
    /// * `sequence_pos` - A boxed slice of positional corrections.
    /// * `sequence_vel` - A boxed slice of velocity corrections.
    /// * `acc_dt` - Acceleration time duration, in seconds.
    /// * `detumble_dt` - Detumbling time duration, in seconds.
    /// * `cost_factor` - The predetermined cost factor for the sequence.
    /// * `res_angle_dev` - The allowed residual angular deviation.
    pub fn new(
        start_i: IndexedOrbitPosition,
        sequence_pos: Box<[Vec2D<I32F32>]>,
        sequence_vel: Box<[Vec2D<I32F32>]>,
        acc_dt: usize,
        detumble_dt: usize,
        cost_factor: I32F32,
        res_angle_dev: I32F32,
    ) -> Self {
        Self {
            start_i,
            sequence_pos,
            sequence_vel,
            acc_dt,
            detumble_dt,
            cost_factor,
            res_angle_dev,
        }
    }

    /// Returns the cost factor for the burn sequence.
    pub fn cost(&self) -> I32F32 { self.cost_factor }

    /// Returns the starting orbital position as `IndexedOrbitPosition` for the sequence.
    pub fn start_i(&self) -> IndexedOrbitPosition { self.start_i }

    /// Returns the sequence of positional corrections.
    pub fn sequence_pos(&self) -> &[Vec2D<I32F32>] { &self.sequence_pos }

    /// Returns the sequence of velocity corrections.
    pub fn sequence_vel(&self) -> &[Vec2D<I32F32>] { &self.sequence_vel }

    /// Returns the detumbling time duration, in seconds.
    pub fn detumble_dt(&self) -> usize { self.detumble_dt }

    /// Returns the acceleration time duration, in seconds.
    pub fn acc_dt(&self) -> usize { self.acc_dt }

    /// Returns the allowed residual angular deviation after the sequence.
    pub fn res_angle_dev(&self) -> I32F32 { self.res_angle_dev }
}
