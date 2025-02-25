use crate::flight_control::common::vec2d::Vec2D;
use fixed::types::I32F32;

pub struct BeaconMeasurement {
    pub pos: Vec2D<I32F32>,
    pub distance: I32F32,
}
