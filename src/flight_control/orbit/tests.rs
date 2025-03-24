use crate::STATIC_ORBIT_VEL;
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::vec2d::{MapSize, Vec2D};
use crate::flight_control::orbit::{ClosedOrbit, OrbitBase};
use fixed::types::I32F32;
use itertools::Itertools;
use num::Zero;
use rand::Rng;

#[test]
fn test_orbit_segments_and_closest() {
    let closed_orbit = init_orbit();
    let segments = closed_orbit.segments();
    println!("Orbit segments: {segments:?}");
    let min_hor_dist = I32F32::MAX;
    let min_vert_dist = I32F32::MAX;
    let mut segments_clone = (*segments).clone();
    segments_clone.retain(|seg| {
        seg.start().x() >= I32F32::zero() && seg.start().x() <= Vec2D::<I32F32>::map_size().x()
    });
    let min_x_vec = segments_clone
        .iter()
        .combinations(2)
        .map(|pair| {
            let dist = (pair[0].start().x() - pair[1].start().x()).abs().round();
            // explicitly return dist and both segments
            (dist, pair[0], pair[1])
        })
        .sorted_by_key(|pair| pair.0)
        .collect::<Vec<_>>();
    let only_dists = min_x_vec.iter().map(|pair| pair.0).collect::<Vec<_>>();
    println!("min_x_dists: {only_dists:?}");
    let rand_pos = get_rand_pos();
    let closest_dist = closed_orbit.get_closest_deviation(rand_pos);
    let prop_on_orbit_pos =
        (rand_pos + Vec2D::from_axis_and_val(closest_dist.0, closest_dist.1)).wrap_around_map();
    assert!(closed_orbit.will_visit(prop_on_orbit_pos));
    println!(
        "Randomized Position: {rand_pos}, Closest Deviation: {closest_dist:?}, Prop on orbit: {prop_on_orbit_pos}"
    );
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
    let rand_dev = if x_dev {
        Vec2D::new(rand_deviation, I32F32::zero())
    } else {
        Vec2D::new(I32F32::zero(), rand_deviation)
    };
    (
        (rand_step_pos + rand_sub_step + rand_dev).wrap_around_map(),
        rand_step_count,
    )
}

fn get_rand_pos() -> Vec2D<I32F32> {
    let mut rng = rand::rng();
    Vec2D::new(
        I32F32::from_num(rng.random_range(0..21600)),
        I32F32::from_num(rng.random_range(0..10800)),
    )
    .round()
}
