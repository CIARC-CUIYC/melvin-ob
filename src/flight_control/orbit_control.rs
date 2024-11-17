use chrono::TimeDelta;
use crate::flight_control::common::Vec2D;

fn position_in_secs<T1, T2>(mut current_pos: Vec2D<T1>, current_speed: Vec2D<T2>, time_delta: TimeDelta)
                            -> Vec2D<T1>
where
    T1: num_traits::real::Real,
    T2: num_traits::real::Real,
{
    current_pos + current_speed * time_delta.num_seconds()
    
    
}