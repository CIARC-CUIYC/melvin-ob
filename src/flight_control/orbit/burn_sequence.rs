use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::orbit::index::IndexedOrbitPosition;

pub struct BurnSequence {
    start_i: IndexedOrbitPosition,
    sequence: Vec<(Vec2D<f32>, Vec2D<f32>)>,
    acc_dt: usize,
    detumble_dt: usize,
    cost_factor: f32,
}

impl BurnSequence {
    pub fn new(
        start_i: IndexedOrbitPosition,
        sequence: Vec<(Vec2D<f32>, Vec2D<f32>)>,
        acc_dt: usize,
        detumble_dt: usize,
        cost_factor: f32,
    ) -> Self {
        Self {
            start_i,
            sequence,
            acc_dt,
            detumble_dt,
            cost_factor
        }
    }

    pub fn cost(&self) -> f32 {
        self.cost_factor
    }

    pub fn start_i(&self) -> IndexedOrbitPosition {self.start_i}

    pub fn sequence(&self) -> &Vec<(Vec2D<f32>, Vec2D<f32>)> {&self.sequence}

    pub fn detumble_dt(&self) -> usize {self.detumble_dt}

    pub fn acc_dt(&self) -> usize {self.acc_dt}
}
