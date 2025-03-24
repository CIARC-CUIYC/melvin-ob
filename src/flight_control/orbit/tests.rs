use fixed::types::I32F32;
use rand::Rng;
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::orbit::{ClosedOrbit, OrbitBase};
use crate::STATIC_ORBIT_VEL;

#[test]
fn test_orbit_segments_and_closest() {
    let init_pos = get_rand_pos();
    let o_b = OrbitBase::test(init_pos, Vec2D::from(STATIC_ORBIT_VEL));
    let closed_orbit = ClosedOrbit::new(o_b, CameraAngle::Narrow).unwrap();
    let segments = closed_orbit.segments();
    println!("Orbit segments: {segments:?}");
    let rand_pos = get_rand_pos();
    let closest_dist = closed_orbit.get_closest_deviation(rand_pos);
    let prop_on_orbit_pos = (rand_pos + Vec2D::from_axis_and_val(closest_dist.0, closest_dist.1)).wrap_around_map();
    assert!(closed_orbit.will_visit(prop_on_orbit_pos));
    println!("Randomized Position: {rand_pos}, Closest Deviation: {closest_dist:?}, Prop on orbit: {prop_on_orbit_pos}");
}


fn get_rand_pos() -> Vec2D<I32F32> {
    let mut rng = rand::rng();
    Vec2D::new(
        I32F32::from_num(rng.random_range(0..21600)),
        I32F32::from_num(rng.random_range(0..10800)),
    )
        .round()
}