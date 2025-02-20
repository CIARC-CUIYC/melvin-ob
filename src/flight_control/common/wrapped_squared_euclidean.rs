use fixed::types::I32F32;
use kiddo::traits::DistanceMetric;
use crate::flight_control::common::bayesian_set::SquareSlice;
use crate::flight_control::common::vec2d::{MapSize, Vec2D};

pub struct WrappedSquaredEuclidean {}

impl DistanceMetric<f64, 2> for WrappedSquaredEuclidean
{
    #[inline]
    fn dist(a: &[f64; 2], b: &[f64; 2]) -> f64 {
        let a_wrapped = Vec2D::from(a).wrap_around_map();
        let b_wrapped = Vec2D::from(b).wrap_around_map();
        let a_fix: Vec2D<I32F32> = Vec2D::from_real(&a_wrapped);
        let b_fix: Vec2D<I32F32> = Vec2D::from_real(&b_wrapped);
        a_fix.unwrapped_to(&b_fix).abs_sq().to_num::<f64>()
    }

    #[inline]
    fn dist1(a: f64, b: f64) -> f64 {
        let map = Vec2D::<f64>::map_size();
        let x_min = map.x() * SquareSlice::MIN_X_ADD_MAP_FACTOR;
        if a > x_min && b > x_min {
            let a_vec: Vec2D<I32F32> = Vec2D::from_real(&Vec2D::new(a, 0.0));
            let b_vec: Vec2D<I32F32> = Vec2D::from_real(&Vec2D::new(b, 0.0));
            let dist = a_vec.unwrapped_to(&b_vec).abs_sq().to_num::<f64>();
            dist
        } else if a < x_min && b < x_min {
            let a_vec: Vec2D<I32F32> = Vec2D::from_real(&Vec2D::new(0.0, a));
            let b_vec: Vec2D<I32F32> = Vec2D::from_real(&Vec2D::new(0.0, b));
            let dist = a_vec.unwrapped_to(&b_vec).abs_sq().to_num::<f64>();
            dist
        } else {
            panic!("[FATAL] Illegal coordinate pair: (x, y) = ({a}, {b})");
        }
    }
}