use super::{bayesian_set::BayesianSet, vec2d::Vec2D};
use crate::STATIC_ORBIT_VEL;
use crate::flight_control::objective::beacon_objective::BeaconMeas;
use chrono::TimeDelta;
use fixed::types::I32F32;
use rand::{Rng, rng};

const MAX_MEASURE_POINTS: usize = 6;
const MEASURE_PERIOD: I32F32 = I32F32::lit("60");
const MELVIN_SIM_STEP: (I32F32, I32F32) = STATIC_ORBIT_VEL;

fn get_d_noisy(d_true: f32) -> f32 {
    let rand_k = rng().random_range(-1.0..=1.0);
    let noise = rand_k * (BayesianSet::K_ADD.to_num::<f32>() + 0.1 * (d_true + 1.0));
    (d_true + noise).max(0.0)
}

#[test]
fn test_bayesian_filter() {
    println!("Running Bayesian Filter Test");
    /*let offset = {
        let map_fixed: Vec2D<I32F32> = Vec2D::map_size();
        let map_size = Vec2D::new(map_fixed.x().to_num::<f32>(), map_fixed.y().to_num::<f32>());
        let x = rng().random_range(0.0..map_size.x()).floor();
        let y = rng().random_range(0.0..map_size.y()).floor();
        Vec2D::new(I32F32::from_num(x), I32F32::from_num(y))
    };
    //println!("Generated offset: {offset}");
    let measure_positions: Vec<Vec2D<I32F32>> = (0..MAX_MEASURE_POINTS)
        .map(|i| {
            (offset + Vec2D::from(MELVIN_SIM_STEP) * I32F32::from_num(i) * MEASURE_PERIOD)
                .wrap_around_map()
                .floor()
        })
        .collect();

    let beacon_pos = {
        let mag = rng().random_range(0.0..=2000.0);
        let angle: f32 = rng().random_range(0.0..=f32::PI() / 2.0);
        let b = Vec2D::new(
            I32F32::from_num((mag * angle.cos()).floor()),
            I32F32::from_num((mag * angle.sin()).floor()),
        );
        (offset + b).round().wrap_around_map()
    };

    let beacon_pos_i32 = Vec2D::new(
        beacon_pos.x().to_num::<i32>(),
        beacon_pos.y().to_num::<i32>(),
    );
    */
    let meas_pos = vec![
        Vec2D::new(I32F32::from_num(271), I32F32::from_num(64)),
        Vec2D::new(I32F32::from_num(466), I32F32::from_num(290)),
        Vec2D::new(I32F32::from_num(661), I32F32::from_num(516)),
        Vec2D::new(I32F32::from_num(856), I32F32::from_num(741)),
        Vec2D::new(I32F32::from_num(1051), I32F32::from_num(967)),
        Vec2D::new(I32F32::from_num(1247), I32F32::from_num(1193)),
        Vec2D::new(I32F32::from_num(1442), I32F32::from_num(1418)),
        Vec2D::new(I32F32::from_num(1637), I32F32::from_num(1644)),
        Vec2D::new(I32F32::from_num(1832), I32F32::from_num(1870)),
        Vec2D::new(I32F32::from_num(2027), I32F32::from_num(2096)),
    ];

    let d_noisy = vec![
        1783.39, 1955.2, 1474.0, 1310.44, 1125.43, 1220.42, 1006.02, 1349.5, 1423.85, 1671.7,
    ];

    //println!("Generated beacon Position: {beacon_pos}");

    let pos = meas_pos[0];
    //let d_true = pos.unwrapped_to(&beacon_pos).abs().to_num::<f32>();
    //println!("STEP 0: {pos}\n\t Distance: {d_true}");
    let noisy = d_noisy[0];
    println!("\t Distance Noisy: {noisy}");
    let meas = BeaconMeas::new(0, pos, f64::from(noisy), TimeDelta::zero());
    let mut bayesian_set = BayesianSet::new(meas);
    let min_guesses = bayesian_set.guess_estimate();
    //println!("\t Minimum Guesses: {min_guesses}");
    //assert!(bayesian_set.is_in_set(beacon_pos_i32));

    let mut meas = 1;
    for (i, pos) in meas_pos.iter().enumerate().skip(1) {
        println!("Working on step {i}");
        //let d_true = pos.unwrapped_to(&beacon_pos).abs().to_num::<f32>();
        //println!("STEP {i}: {pos}\n\t Distance: {d_true}");
        //if d_true > BayesianSet::MAX_DIST {
        //println!("\t Distance too large, skipping");
        //    continue;
        // };
        let noisy = d_noisy[i];
        println!("\t Distance Noisy: {noisy}");
        let b_meas = BeaconMeas::new(0, *pos, f64::from(noisy), TimeDelta::zero());
        bayesian_set.update(&b_meas);
        let min_guesses = bayesian_set.guess_estimate();
        //println!("\t Minimum Guesses: {min_guesses}");
        //assert!(bayesian_set.is_in_set(beacon_pos_i32));
        meas += 1;
    }

    println!("Got {meas} measurements, passed!");
    let centers = bayesian_set.pack_perfect_circles();
    println!("\t {} possible centers: ", centers.len());
    for (i, center) in centers.iter().enumerate() {
        let c_fix = Vec2D::new(I32F32::from_num(center.x()), I32F32::from_num(center.y()));
        println!("\t Center {i}: {c_fix}");

        /*let hits = c_fix.unwrapped_to(&beacon_pos).abs().to_num::<f32>()
            < BayesianSet::MAX_RES_UNCERTAINTY_RAD;
        if hits {
            println!("\t Finished. Hits on {i}. try with center: {center}");
            break;
        }*/
    }
}
