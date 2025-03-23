use super::task_controller::TaskController;
use crate::{error, info, log, STATIC_ORBIT_VEL};
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::orbit::IndexedOrbitPosition;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;
use rand::Rng;

const STATIC_PERIOD: usize = 54000;

fn get_start_pos() -> IndexedOrbitPosition {
    IndexedOrbitPosition::new(0, STATIC_PERIOD, get_rand_pos())
}

fn get_rand_pos() -> Vec2D<I32F32> {
    let mut rng = rand::rng();
    Vec2D::new(
        I32F32::from_num(rng.random_range(0..21600)),
        I32F32::from_num(rng.random_range(0..10800)),
    ).round()
}

fn get_rand_end_t() -> DateTime<Utc> {
    const MIN_SECS: i64 = 4 * 3600;
    const MAX_SECS: i64 = 8 * 3600;
    let mut rng = rand::rng();
    let now = Utc::now();
    let rand_secs = rng.random_range(MIN_SECS..MAX_SECS);
    now + TimeDelta::seconds(rand_secs)
}

fn get_rand_fuel() -> I32F32 {
    const MIN_FUEL: f32 = 15.0;
    const MAX_FUEL: f32 = 100.0;
    let mut rng = rand::rng();
    I32F32::from_num(rng.random_range(MIN_FUEL..MAX_FUEL))
}

#[tokio::test]
async fn test_single_target_burn_calculator() {
    loop {
        info!("Running Single Target Burn Calculator Test");
        let mock_start_point = get_start_pos();
        let mock_obj_point = get_rand_pos();
        let mock_end_t = get_rand_end_t();
        let mock_fuel_left = get_rand_fuel();
        let res = TaskController::calculate_single_target_burn_sequence(
            mock_start_point,
            Vec2D::from(STATIC_ORBIT_VEL),
            mock_obj_point,
            mock_end_t,
            mock_fuel_left,
        ).await.unwrap();
        let exit_burn = res.sequence();
        let entry_pos = exit_burn.sequence_pos().first().unwrap();
        let exit_pos = *exit_burn.sequence_pos().last().unwrap();
        let exit_vel = *exit_burn.sequence_vel().last().unwrap();
        let entry_t = exit_burn.start_i().t().format("%H:%M:%S").to_string();
        let detumble_dt = I32F32::from_num(exit_burn.detumble_dt());
        let acc_dt = I32F32::from_num(exit_burn.acc_dt());
        let est = (exit_pos + exit_vel * detumble_dt).wrap_around_map();
        // Check if the estimated Y position is close to the target Y
        let hit_target = est.to(res.target_pos()).abs() < detumble_dt * (I32F32::lit("0.002") * detumble_dt);
        if hit_target {
            info!("Test successfull. Acc for {acc_dt}s, detumble for {detumble_dt}s!")
        } else {
            error!("Test failed.");
            info!("Calculated Burn Sequence for mocked Zoned Objective at: {mock_obj_point}, due at {mock_end_t}");
            log!("Entry at {entry_t}, Position will be {entry_pos}");
            log!("Exit after {acc_dt}s, Position will be {exit_pos}");
            log!("Exit Velocity will be {exit_vel} aiming for target at {mock_obj_point}. Detumble time is {detumble_dt}s.");
            log!("Whole BS: {:?}", res);
            return;
        }
    }
}

/*
fn get_rand_detumple_point(base: Vec2D<I32F32>) -> Vec2D<I32F32> {
    let mut rng = rand::rng();
    let angle = I32F32::from_num(rng.random_range(0..360)) * I32F32::PI / I32F32::from_num(180);
    let rad = I32F32::from_num(rng.random_range(700..4000));
    let offset_x = rad * I32F32::from_num(angle.to_num::<f32>().cos());
    let offset_y = rad * I32F32::from_num(angle.to_num::<f32>().sin());
    base + Vec2D::new(offset_x, offset_y).round().wrap_around_map()
}

#[test]
fn test_zo_retrieval_burn_calculator() {
    info!("Running ZO Retrieval Burn Calculator Test");
    let mock_start_point = get_start_pos();
    let mock_obj_point = get_rand_detumple_point(get_rand_pos());
    let direction_to = mock_start_point.pos().unwrapped_to(&mock_obj_point);
    let mut rng = rand::rng();
    let random_angle_deg = rng.random_range(-1.0..=1.0);
    let mut rotated_dir = direction_to;
    rotated_dir.rotate_by(I32F32::from_num(random_angle_deg));
    let (random_vel, _) = FlightComputer::trunc_vel(rotated_dir.normalize() * I32F32::from_num(rng.random_range(5.0..9.0)));
    let dt = rotated_dir.abs() / random_vel.abs();
    let deviation = direction_to - random_vel * dt;    
    let due = Utc::now() +  TimeDelta::seconds(dt.to_num::<i64>());
    info!("Calculated Burn for mocked Target: {mock_obj_point}, with base point at {}", mock_start_point.pos());
    log!("Current velocity is randomized at {random_vel}, arrival will be in {dt}s, deviation will be {deviation}");
    let res = TaskController::calculate_orbit_correction_burn(random_vel, deviation, due);
    log!("Found burn with {}s of hold time! Expected resulting Deviation will be {}", res.1, res.2);
    log!("Velocity change sequence is {:?}", res.0);
}
*/