use super::task_controller::TaskController;
use crate::{info, log, STATIC_ORBIT_VEL};
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
    )
}

fn get_rand_end_t() -> DateTime<Utc> {
    const MIN_SECS: i64 = 4 * 3600;
    const MAX_SECS: i64 = 8 * 3600;
    let mut rng = rand::rng();
    let now = Utc::now();
    let rand_secs = rng.random_range(MIN_SECS..MAX_SECS);
    now + TimeDelta::seconds(rand_secs)
}

#[tokio::test]
async fn test_single_target_burn_scheduler() {
    println!("Running Bayesian Filter Test");
    let mock_start_point = get_start_pos();
    let mock_obj_point = get_rand_pos();
    let mock_end_t = get_rand_end_t();
    let exit_burn = TaskController::calculate_single_target_burn_sequence(
        mock_start_point,
        Vec2D::from(STATIC_ORBIT_VEL),
        mock_obj_point,
        mock_end_t,
    ).await.unwrap();
    let entry_pos = exit_burn.sequence_pos().first().unwrap();
    let exit_pos = exit_burn.sequence_pos().last().unwrap();
    let entry_t = exit_burn.start_i().t().format("%H:%M:%S").to_string();
    let exit_vel = exit_burn.sequence_vel().last().unwrap();
    info!("Calculated Burn Sequence for mocked Zoned Objective at: {mock_obj_point}, due at {mock_end_t}");
    log!("Entry at {entry_t}, Position will be {entry_pos}");
    log!("Exit after {}s, Position will be {exit_pos}", exit_burn.acc_dt());
    log!("Exit Velocity will be {exit_vel} aiming for target at {mock_obj_point}. Detumble time is {}s.",  exit_burn.detumble_dt());
    log!("Whole BS: {:?}", exit_burn);
}
