use crate::flight_control::{common::vec2d::Vec2D, orbit::index::IndexedOrbitPosition};
use fixed::types::I32F32;

#[derive(Debug, Clone)]
pub struct BurnSequence {
    start_i: IndexedOrbitPosition,
    sequence_pos: Box<[Vec2D<I32F32>]>,
    sequence_vel: Box<[Vec2D<I32F32>]>,
    acc_dt: usize,
    detumble_dt: usize,
    cost_factor: I32F32,
    res_angle_dev: I32F32,
}

impl BurnSequence {
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

    pub fn cost(&self) -> I32F32 { self.cost_factor }

    pub fn start_i(&self) -> IndexedOrbitPosition { self.start_i }

    pub fn sequence_pos(&self) -> &[Vec2D<I32F32>] { &self.sequence_pos }

    pub fn sequence_vel(&self) -> &[Vec2D<I32F32>] { &self.sequence_vel }

    pub fn detumble_dt(&self) -> usize { self.detumble_dt }

    pub fn acc_dt(&self) -> usize { self.acc_dt }

    pub fn res_angle_dev(&self) -> I32F32 { self.res_angle_dev }
}
