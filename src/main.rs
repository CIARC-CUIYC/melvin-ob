mod console_communication;
mod flight_control;
mod http_handler;

use crate::console_communication::console_endpoint::ConsoleEndpoint;
use crate::flight_control::flight_state::FlightState;
use crate::flight_control::orbit::{
    closed_orbit::{ClosedOrbit, OrbitUsabilityError},
    orbit_base::OrbitBase,
};
use crate::flight_control::task::base_task::BaseTask;
use crate::flight_control::task_controller::TaskController;
use crate::flight_control::{
    camera_controller::CameraController, camera_state::CameraAngle, flight_computer::FlightComputer,
};
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_request::observation_get::ObservationRequest;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use chrono::{DateTime, TimeDelta};
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::Arc;
use tokio::sync::{Mutex, Notify};
use tokio::task::JoinHandle;

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

const LOG_POS: bool = true;

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let client = Arc::new(HTTPClient::new("http://localhost:33000"));
    let console_endpoint = Arc::new(ConsoleEndpoint::start());
    if LOG_POS {
        let thread_client = Arc::clone(&client);
        tokio::spawn(async move { log_pos(thread_client).await });
    }

    let t_cont = TaskController::new();
    let c_cont = CameraController::from_file(BIN_FILEPATH).await.unwrap_or_else(|e| {
        println!("[WARN] Failed to read from binary file: {e}");
        CameraController::new()
    });
    let c_cont_locked = Arc::new(Mutex::new(c_cont));
    let mut f_cont = FlightComputer::new(Arc::clone(&client), &t_cont).await;

    let c_orbit: ClosedOrbit = {
        f_cont.reset().await;
        f_cont.set_vel_ff(STATIC_ORBIT_VEL.into(), false).await;
        f_cont.set_angle(CONST_ANGLE).await;
        ClosedOrbit::new(OrbitBase::new(&f_cont), CameraAngle::Wide).unwrap_or_else(|e| match e {
            OrbitUsabilityError::OrbitNotClosed => panic!("[FATAL] Static orbit is not closed"),
            OrbitUsabilityError::OrbitNotEnoughOverlap => {
                panic!("[FATAL] Static orbit is not overlapping enough")
            }
        })
    };
    let f_cont_locked = Arc::new(Mutex::new(f_cont));

    let img_dt = c_orbit.max_image_dt();
    let orbit_s_end = c_orbit.base_orbit_ref().start_timestamp()
        + chrono::Duration::seconds(c_orbit.period().0 as i64);

    let c_orbit_arc = Arc::new(c_orbit);
    let t_cont_locked = Arc::new(Mutex::new(t_cont));

    // orbit
    loop {
        let f_cont_clone = Arc::clone(&f_cont_locked);
        let c_orbit_arc_clone = Arc::clone(&c_orbit_arc);
        let t_cont_clone = Arc::clone(&t_cont_locked);
        let schedule_join_handle = tokio::spawn(async move {
            let mut t_cont = t_cont_clone.lock().await;
            t_cont.schedule_optimal_orbit(&c_orbit_arc_clone, f_cont_clone).await;
        });
        let current_state = {
            let mut f_cont = f_cont_locked.lock().await;
            f_cont.update_observation().await;
            f_cont.state()
        };
        let acq_phase = if current_state == FlightState::Acquisition {
            let end_time = chrono::Utc::now() + chrono::Duration::seconds(10000);
            Some(handle_acquisition(
                &f_cont_locked,
                &c_cont_locked,
                end_time,
                img_dt,
                CONST_ANGLE,
            ))
        } else {
            None
        };

        schedule_join_handle.await.ok();
        if let Some(h) = acq_phase {
            h.1.notify_one();
            h.0.await.ok();
        }

        while let Some(task) = t_cont_locked.lock().await.sched_arc().pop() {
            let task_type = task.task_type();
            let due_time = task.dt().time_left();
            if due_time > DT_0 {
                let mut f_cont = f_cont_locked.lock().await;
                f_cont.make_ff_call(due_time.to_std().unwrap()).await;
            }
            match task_type {
                BaseTask::TASKImageTask(_) => {
                    todo!()
                }
                BaseTask::TASKSwitchState(switch) => match switch.target_state() {
                    FlightState::Acquisition => {
                        let mut f_cont = f_cont_locked.lock().await;
                        f_cont.state_change_ff(FlightState::Acquisition).await;
                        if let Some(next) = t_cont_locked.lock().await.sched_arc().peek() {
                            let task_due = next.dt().time_left();
                            let acq_phase = handle_acquisition(
                                &f_cont_locked,
                                &c_cont_locked,
                                next.dt().get_end(),
                                img_dt,
                                CONST_ANGLE,
                            );
                            tokio::time::sleep(task_due.to_std().unwrap()).await;
                            acq_phase.1.notify_one();
                            acq_phase.0.await.ok();
                            // TODO: export png here
                        }
                    }
                    FlightState::Charge => {
                        let mut f_cont = f_cont_locked.lock().await;
                        f_cont.state_change_ff(FlightState::Charge).await;
                    }
                    FlightState::Comms => {}
                    _ => {
                        panic!("[FATAL] Illegal target state!")
                    }
                },
            }
            // TODO: perform optimal orbit until objective notification
        }
    }

    /* if chrono::Utc::now() > orbit_s_end {
        println!(
            "[LOG] First orbit over, position: {:?}",
            f_cont.current_pos()
        );
        break;
    }*/

    //c_cont.save_png_to(BIN_FILEPATH).await.unwrap();
}

fn handle_acquisition(
    f_cont_locked: &Arc<Mutex<FlightComputer>>,
    c_cont_locked: &Arc<Mutex<CameraController>>,
    end_time: DateTime<chrono::Utc>,
    img_dt: f32,
    angle: CameraAngle,
) -> (
    JoinHandle<()>,
    Arc<Notify>,
    Arc<Mutex<DateTime<chrono::Utc>>>,
) {
    let f_cont_locked_arc_clone = Arc::clone(f_cont_locked);
    let end_time_locked = Arc::new(Mutex::new(end_time));
    let end_time_cloned = Arc::clone(&end_time_locked);
    let c_cont_cloned = Arc::clone(c_cont_locked);
    let last_image_notify = Arc::new(Notify::new());
    let last_image_notify_cloned = Arc::clone(&last_image_notify);

    let handle = tokio::spawn(async move {
        CameraController::execute_acquisition_cycle(
            c_cont_cloned,
            f_cont_locked_arc_clone,
            end_time_cloned,
            last_image_notify_cloned,
            img_dt,
            angle,
        )
        .await;
    });
    (handle, last_image_notify, end_time_locked)
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
                for (x, y) in &positions {
                    writeln!(f, "{x},{y}").unwrap();
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
