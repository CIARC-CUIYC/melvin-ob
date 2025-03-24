use fixed::types::I32F32;
use num::Zero;
use rand::Rng;
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::orbit::{ClosedOrbit, OrbitBase};
use crate::STATIC_ORBIT_VEL;
use itertools::Itertools;

#[test]
fn test_orbit_segments_and_closest() {
    let closed_orbit = init_orbit();
    let segments = closed_orbit.segments();
    println!("Orbit segments: {segments:?}");
    let min_hor_dist = I32F32::MAX;
    let min_vert_dist = I32F32::MAX;
    let min_x = segments.iter().combinations(2).map(|pair| {
        let mut dist = (pair[0].start().y() - pair[1].start().y()).abs();
        if dist < I32F32::from_num(50) {
            dist = I32F32::MAX;
        }
        // explicitly return dist and both segments
        (dist, pair[0], pair[1])

    }).min_by_key(|pair| pair.0).unwrap();
    println!("min_x_dist: {}, with p1: {} and p2: {}", min_x.0, min_x.1.start(), min_x.2.start());
    println!("ends aswell: p1 {}, p2 {}", min_x.1.end(), min_x.2.end());
    let rand_pos = get_rand_pos();
    let closest_dist = closed_orbit.get_closest_deviation(rand_pos);
    let prop_on_orbit_pos = (rand_pos + Vec2D::from_axis_and_val(closest_dist.0, closest_dist.1)).wrap_around_map();
    assert!(closed_orbit.will_visit(prop_on_orbit_pos));
    println!("Randomized Position: {rand_pos}, Closest Deviation: {closest_dist:?}, Prop on orbit: {prop_on_orbit_pos}");
}

#[test]
fn test_orbit_get_i() {
    let closed_orbit = init_orbit();
    let (pos, i) = get_rand_orbit_pos(&closed_orbit);
    let guess_i = closed_orbit.get_i(pos).unwrap();
    let min_i = i.saturating_sub(1);
    let max_i = (i + 1) % closed_orbit.period().0.to_num::<usize>();
    if !(guess_i > min_i && guess_i < max_i) {
        println!("Actual i: {i}, pos: {pos}, guess_i: {guess_i}");
    }
}

fn init_orbit() -> ClosedOrbit {
    let init_pos = get_rand_pos();
    let o_b = OrbitBase::test(init_pos, Vec2D::from(STATIC_ORBIT_VEL));
    ClosedOrbit::new(o_b, CameraAngle::Narrow).unwrap()
}


fn get_rand_orbit_pos(orbit: &ClosedOrbit) -> (Vec2D<I32F32>, usize) {
    let mut rng = rand::rng();
    let rand_step_count = rng.random_range(0..orbit.period().0.to_num::<usize>());
    let step = *orbit.base_orbit_ref().vel();
    let start = *orbit.base_orbit_ref().fp();
    let rand_step_pos = (start + step * I32F32::from_num(rand_step_count)).wrap_around_map();
    let rand_sub_step = step * I32F32::from_num(rng.random_range(0.0..1.0));
    let rand_deviation = I32F32::from_num(rng.random_range(0.0..1.0));
    let x_dev = rng.random_bool(0.5);
    let rand_dev = if x_dev == true {
        Vec2D::new(rand_deviation, I32F32::zero())
    } else {
        Vec2D::new(I32F32::zero(), rand_deviation)
    };
    ((rand_step_pos + rand_sub_step + rand_dev).wrap_around_map(), rand_step_count)
}

fn get_rand_pos() -> Vec2D<I32F32> {
    let mut rng = rand::rng();
    Vec2D::new(
        I32F32::from_num(rng.random_range(0..21600)),
        I32F32::from_num(rng.random_range(0..10800)),
    )
        .round()
}