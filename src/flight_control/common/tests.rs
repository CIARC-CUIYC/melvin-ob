use crate::flight_control::common::bayesian_set::BayesianSet;
use crate::flight_control::common::vec2d::{MapSize, Vec2D};
use crate::STATIC_ORBIT_VEL;
use fixed::types::I32F32;
use num::traits::FloatConst;
use rand::{rng, Rng};
use crate::flight_control::objective::beacon_objective::BeaconMeas;

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
    for i_gl in 0..20 {
        let offset = {
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
                    .wrap_around_map().floor()
            })
            .collect();

        let beacon_pos = {
            let mag = rng().random_range(0.0..=2000.0);
            let angle: f32 = rng().random_range(0.0..=f32::PI() * 2.0);
            let b = Vec2D::new(
                I32F32::from_num((mag * angle.cos()).floor()),
                I32F32::from_num((mag * angle.sin()).floor()),
            );
            (offset + b).round().wrap_around_map()
        };


        let beacon_pos_i32 = Vec2D::new(beacon_pos.x().to_num::<i32>(), beacon_pos.y().to_num::<i32>());
        println!("Generated beacon Position: {beacon_pos}");

        let pos = measure_positions[0];
        let d_true = pos.unwrapped_to(&beacon_pos).abs().to_num::<f32>();
        //println!("STEP 0: {pos}\n\t Distance: {d_true}");
        let d_noisy = get_d_noisy(d_true);
        //println!("\t Distance Noisy: {d_noisy}");
        let meas = BeaconMeas::new(0, pos, d_noisy as f64);
        let mut bayesian_set = BayesianSet::new(meas);
        let min_guesses = bayesian_set.guess_estimate();
        //println!("\t Minimum Guesses: {min_guesses}");
        assert!(bayesian_set.is_in_set(beacon_pos_i32));

        let mut meas = 1;
        for (i, pos) in measure_positions.iter().enumerate().skip(1) {
            let d_true = pos.unwrapped_to(&beacon_pos).abs().to_num::<f32>();
            //println!("STEP {i}: {pos}\n\t Distance: {d_true}");
            if d_true > BayesianSet::MAX_DIST {
                //println!("\t Distance too large, skipping");
                continue;
            };
            let d_noisy = get_d_noisy(d_true);
            //println!("\t Distance Noisy: {d_noisy}");
            let b_meas = BeaconMeas::new(0, *pos, d_noisy as f64);
            bayesian_set.update(b_meas);
            let min_guesses = bayesian_set.guess_estimate();
            //println!("\t Minimum Guesses: {min_guesses}");
            assert!(bayesian_set.is_in_set(beacon_pos_i32));
            meas += 1;
        }

        println!("Got {meas} measurements, passed!");
        let centers = bayesian_set.pack_perfect_circles();
        println!("\t {} possible centers: ", centers.len());
        for (i, center) in centers.iter().enumerate() {
            let c_fix = Vec2D::new(I32F32::from_num(center.x()), I32F32::from_num(center.y()));
            let hits = c_fix.unwrapped_to(&beacon_pos).abs().to_num::<f32>() < BayesianSet::MAX_RES_UNCERTAINTY_RAD;
            if hits {
                println!("\t Iteration {i_gl} finished. Hits on {i}. try with center: {center}");
                break;
            }
        }
    }
    
}
