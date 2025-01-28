use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::orbit::index::IndexedOrbitPosition;

#[derive(Debug)]
pub struct BurnSequence {
    start_i: IndexedOrbitPosition,
    sequence_pos: Box<[Vec2D<f32>]>,
    sequence_vel: Box<[Vec2D<f32>]>,
    acc_dt: usize,
    detumble_dt: usize,
    cost_factor: f32,
    res_angle_dev: f32,
}

impl BurnSequence {
    pub fn new(
        start_i: IndexedOrbitPosition,
        sequence_pos: Box<[Vec2D<f32>]>,
        sequence_vel: Box<[Vec2D<f32>]>,
        acc_dt: usize,
        detumble_dt: usize,
        cost_factor: f32,
        res_angle_dev: f32,
    ) -> Self {
        Self {
            start_i,
            sequence_pos,
            sequence_vel,
            acc_dt,
            detumble_dt,
            cost_factor,
            res_angle_dev
        }
    }

    pub fn cost(&self) -> f32 {
        self.cost_factor
    }

    pub fn start_i(&self) -> IndexedOrbitPosition {self.start_i}

    pub fn sequence_pos(&self) -> &Box<[Vec2D<f32>]> {&self.sequence_pos}

    pub fn sequence_vel(&self) -> &Box<[Vec2D<f32>]> {&self.sequence_vel}

    pub fn detumble_dt(&self) -> usize {self.detumble_dt}

    pub fn acc_dt(&self) -> usize {self.acc_dt}
    
    pub fn res_angle_dev(&self) -> f32 {self.res_angle_dev}
}
