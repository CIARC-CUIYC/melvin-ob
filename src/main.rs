mod console_communication;
mod flight_control;
mod http_handler;

use crate::console_communication::console_endpoint::{ConsoleEndpoint, ConsoleEndpointEvent};
use crate::console_communication::melvin_messages;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::flight_computer::ChargeCommand::TargetCharge;
use crate::flight_control::flight_state::FlightState;
use crate::flight_control::orbit::{
    closed_orbit::{ClosedOrbit, OrbitUsabilityError},
    orbit_base::OrbitBase,
};
use crate::flight_control::task_controller::TaskController;
use crate::flight_control::{
    camera_controller::CameraController, camera_state::CameraAngle, flight_computer::FlightComputer,
};
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_request::observation_get::ObservationRequest;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use chrono::TimeDelta;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::Arc;
use tokio::time::{sleep, Duration, Instant};

const FUEL_RESET_THRESHOLD: f32 = 20.0;
const MIN_PX_LEFT_FACTOR: f32 = 0.05;
const ACCEL_FACTOR: f32 = 0.6;
const CHARGE_TIME_THRESHOLD: TimeDelta = TimeDelta::seconds(370);
const DT_0: TimeDelta = TimeDelta::seconds(0);

const STATIC_ORBIT_VEL: (f32, f32) = (6.4f32, 7.4f32);
pub const MIN_BATTERY_THRESHOLD: f32 = 10.0;
pub const MAX_BATTERY_THRESHOLD: f32 = 100.0;
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;
const BIN_FILEPATH: &str = "camera_controller_narrow.bin";
const ACQUISITION_DISCHARGE_PER_S: f32 = 0.1;
const CHARGE_CHARGE_PER_S: f32 = 0.1;

const SIM_TIMESTEP: f32 = 0.5;

const LOG_POS: bool = true;

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let client = Arc::new(HTTPClient::new("http://localhost:33000"));
    let console_endpoint = Arc::new(ConsoleEndpoint::start());
    let t_cont = TaskController::new();
    let mut f_cont = FlightComputer::new(Arc::clone(&client), &t_cont).await;

    f_cont.reset().await;
    f_cont.state_change_ff(FlightState::Charge).await;

    log_expected_and_actual_pos(Arc::clone(&client), f_cont).await;

    //if LOG_POS {
    //    let thread_client = Arc::clone(&client);
    //    tokio::spawn(async move { log_pos(thread_client).await });
    //}

    //let mut t_cont = TaskController::new();
    //let mut c_cont = CameraController::from_file(BIN_FILEPATH).await.unwrap_or_else(|e| {
    //    println!("[WARN] Failed to read from binary file: {e}");
    //    CameraController::new()
    //});
    //let mut f_cont = FlightComputer::new(&*client, &t_cont).await;

    //f_cont.reset().await;
    //f_cont.set_vel_ff(STATIC_ORBIT_VEL.into(), false).await;
    //f_cont.set_angle(CONST_ANGLE).await;

    //let c_orbit = match ClosedOrbit::new(OrbitBase::new(&f_cont), CameraAngle::Wide) {
    //    Ok(orbit) => orbit,
    //    Err(e) => match e {
    //        OrbitUsabilityError::OrbitNotClosed => panic!("[FATAL] Static orbit is not closed"),
    //        OrbitUsabilityError::OrbitNotEnoughOverlap => {
    //            panic!("[FATAL] Static orbit is not overlapping enough")
    //        }
    //    },
    //};

    //let img_dt = c_orbit.max_image_dt();
    //let mut orbit_s_end =
    //    c_orbit.base_orbit_ref().start_timestamp() + chrono::Duration::seconds(c_orbit.period().0 as i64);

    //t_cont.schedule_optimal_orbit(&c_orbit, &mut f_cont);

    //// first orbit-behaviour loop
    //loop {
    //    let mut break_flag = false;
    //    loop {
    //        c_cont.shoot_image_to_buffer(&client, &mut f_cont, CONST_ANGLE).await.unwrap();

    //        if break_flag {
    //            break;
    //        }

    //        let secs_to_min_batt =
    //            (f_cont.current_battery() - MIN_BATTERY_THRESHOLD) / ACQUISITION_DISCHARGE_PER_S;
    //        let secs_to_orbit_end = (orbit_s_end - chrono::Utc::now()).num_seconds() as u64;
    //        let mut sleep_time = img_dt as u64;

    //        if secs_to_min_batt < img_dt {
    //            sleep_time = secs_to_min_batt as u64;
    //            break_flag = true;
    //        }
    //        if secs_to_orbit_end < sleep_time {
    //            sleep_time = secs_to_orbit_end;
    //            break_flag = true;
    //        }

    //        let skipped_time =
    //            f_cont.make_ff_call(std::time::Duration::from_secs(sleep_time)).await;
    //        orbit_s_end -= TimeDelta::seconds(skipped_time);

    //        f_cont.update_observation().await;
    //    }

    //    if chrono::Utc::now() > orbit_s_end {
    //        println!(
    //            "[LOG] First orbit over, position: {:?}",
    //            f_cont.current_pos()
    //        );
    //        break;
    //    };

    //    tokio::join!(
    //        c_cont.save_png_to(BIN_FILEPATH),
    //        f_cont.charge_until(TargetCharge(MAX_BATTERY_THRESHOLD))
    //    )
    //    .0
    //    .unwrap();
    //}
}

async fn log_pos(http_handler: Arc<HTTPClient>) {
    let mut positions: Vec<(i32, i32)> = Vec::new();
    let mut next_save = chrono::Utc::now() + chrono::Duration::seconds(300);
    loop {
        if let Ok(obs) = (ObservationRequest {}.send_request(&http_handler).await) {
            let pos = (i32::from(obs.pos_x()), i32::from(obs.pos_y()));
            positions.push(pos);
        }
        if next_save < chrono::Utc::now() {
            if let Ok(mut f) = OpenOptions::new().create(true).append(true).open("pos.csv") {
                for (x, y) in positions.iter() {
                    writeln!(f, "{},{}", x, y).unwrap();
                }
                next_save = chrono::Utc::now() + chrono::Duration::seconds(300);
                positions.clear();
            } else {
                println!("[WARN] Failed to open pos.csv for writing!");
                continue;
            }
        }
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}

async fn log_expected_and_actual_pos(client: Arc<HTTPClient>, mut f_cont: FlightComputer) {
    let mut actual_pos: Vec<Vec2D<u32>> = Vec::new();
    let mut expected_pos: Vec<Vec2D<f32>> = Vec::new();
    let mut current_vel: Vec2D<f32> = Vec2D::new(0.0, 0.0);
    let mut greatest_distance = 0.0;

    let mut last_iteration_time = Instant::now();
    loop {
        match (ObservationRequest {}.send_request(&client).await) {
            Ok(obs) => {
                current_vel = (obs.vel_x(), obs.vel_y()).into();
                let current_pos = (u32::from(obs.pos_x()), u32::from(obs.pos_y())).into();
                actual_pos.push(current_pos);
                let current_expected_pos = {
                    if expected_pos.is_empty() {
                        let temp = (current_pos.x() as f32, current_pos.y() as f32).into();
                        expected_pos.push(temp);
                        temp
                    } else {
                        let elapsed_time = (Instant::now() - last_iteration_time).as_secs_f32();
                        last_iteration_time = Instant::now();
                        let mut temp = *expected_pos.last().unwrap() + current_vel * elapsed_time;
                        temp = temp.wrap_around_map();
                        expected_pos.push(temp);
                        temp
                    }
                };

                let distance = current_expected_pos.to(&current_pos.cast()).abs();
                greatest_distance = distance.max(greatest_distance);

                println!(
                    "Actual Position: ({}, {}), Expected Position: ({:.2}, {:.2}), Distance: ({:.2}), Greatest Distance ({:.2})",
                    current_pos.x(),
                    current_pos.y(),
                    current_expected_pos.x(),
                    current_expected_pos.y(),
                    distance,
                    greatest_distance
                );
            }
            Err(_) => {
                println!("[ERROR] HTTP Error");
            }
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}
