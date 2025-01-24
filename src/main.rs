#![allow(dead_code, unused)]
mod console_communication;
mod flight_control;
mod http_handler;

use crate::console_communication::console_endpoint::ConsoleEndpoint;
use crate::flight_control::{
    camera_controller::CameraController,
    camera_state::CameraAngle,
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::{
        closed_orbit::{ClosedOrbit, OrbitUsabilityError},
        orbit_base::OrbitBase,
    },
    task::base_task::BaseTask,
    task_controller::TaskController,
};
use crate::http_handler::{
    http_client::HTTPClient,
    http_request::{observation_get::ObservationRequest, request_common::NoBodyHTTPRequestType},
};
use chrono::{DateTime, TimeDelta};
use std::{fs::OpenOptions, io::Write, sync::Arc};
use tokio::{
    sync::{Mutex, Notify, RwLock},
    task::JoinHandle,
};

const DT_0: TimeDelta = TimeDelta::seconds(0);

const STATIC_ORBIT_VEL: (f32, f32) = (6.4f32, 7.4f32);
pub const MIN_BATTERY_THRESHOLD: f32 = 10.0;
pub const MAX_BATTERY_THRESHOLD: f32 = 100.0;
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;
const BIN_FILEPATH: &str = "camera_controller_narrow.bin";

const LOG_POS: bool = false;

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::too_many_lines)]
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
    let c_cont_lock = Arc::new(Mutex::new(c_cont));
    let f_cont_lock = Arc::new(RwLock::new(FlightComputer::new(Arc::clone(&client)).await));

    let f_cont_lock_clone = Arc::clone(&f_cont_lock);
    tokio::spawn(async move {
        loop {
            {
                let mut f_cont = f_cont_lock_clone.write().await;
                (*f_cont).update_observation().await;
            };
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        }
    });

    let c_orbit: ClosedOrbit = {
        f_cont_lock.read().await.reset().await;
        FlightComputer::set_state_wait(&f_cont_lock, FlightState::Acquisition).await;
        FlightComputer::set_vel_wait(&f_cont_lock, STATIC_ORBIT_VEL.into()).await;
        FlightComputer::set_angle_wait(&f_cont_lock, CONST_ANGLE).await;
        let f_cont = f_cont_lock.read().await;
        ClosedOrbit::new(OrbitBase::new(&f_cont), CameraAngle::Wide).unwrap_or_else(|e| match e {
            OrbitUsabilityError::OrbitNotClosed => panic!("[FATAL] Static orbit is not closed"),
            OrbitUsabilityError::OrbitNotEnoughOverlap => {
                panic!("[FATAL] Static orbit is not overlapping enough")
            }
        })
    };

    let img_dt = c_orbit.max_image_dt();
    let orbit_s_end = c_orbit.base_orbit_ref().start_timestamp()
        + chrono::Duration::seconds(c_orbit.period().0 as i64);

    let c_orbit_arc = Arc::new(c_orbit);
    let t_cont_lock = Arc::new(Mutex::new(t_cont));

    // orbit
    loop {
        let f_cont_clone = Arc::clone(&f_cont_lock);
        let c_orbit_arc_clone = Arc::clone(&c_orbit_arc);
        let t_cont_clone = Arc::clone(&t_cont_lock);
        let schedule_join_handle = tokio::spawn(async move {
            let mut t_cont = t_cont_clone.lock().await;
            t_cont.schedule_optimal_orbit(&c_orbit_arc_clone, f_cont_clone).await;
        });
        let current_state = f_cont_lock.read().await.state();

        let acq_phase = if current_state == FlightState::Acquisition {
            let end_time = chrono::Utc::now() + chrono::Duration::seconds(10000);
            Some(handle_acquisition(
                &f_cont_lock,
                &c_cont_lock,
                end_time,
                img_dt,
                CONST_ANGLE,
                false,
            ))
        } else {
            None
        };

        schedule_join_handle.await.ok();
        if let Some(h) = acq_phase {
            h.1.notify_one();
            h.0.await.ok();
        }

        let mut phases = 0;

        while let Some(task) = {
            let start = chrono::Utc::now();
            let t_cont = t_cont_lock.lock().await;
            let sched = t_cont.sched_arc();
            let mut sched_lock = sched.lock().await;
            sched_lock.pop_front()
        } {
            phases += 1;
            let task_type = task.task_type();
            let due_time = task.dt().time_left();
            println!(
                "[INFO] Iteration {phases}: {task_type} in  {} s!",
                due_time.num_seconds()
            );

            let current_state = { f_cont_lock.read().await.state() };
            if current_state == FlightState::Acquisition && due_time > DT_0 {
                let acq_phase = handle_acquisition(
                    &f_cont_lock,
                    &c_cont_lock,
                    task.dt().get_end(),
                    img_dt,
                    CONST_ANGLE,
                    true,
                );
                acq_phase.0.await.ok();
            } else if current_state == FlightState::Charge && due_time > DT_0 {
                let task_due = task.dt().time_left();
                FlightComputer::wait_for_duration(task_due.to_std().unwrap()).await;
            } else {
                panic!("[FATAL] Illegal state ({current_state}) or too little time left for task ({due_time})!")
            }
            match task_type {
                BaseTask::TASKImageTask(_) => {
                    todo!()
                }
                BaseTask::TASKSwitchState(switch) => match switch.target_state() {
                    FlightState::Acquisition => {
                        FlightComputer::set_state_wait(&f_cont_lock, FlightState::Acquisition)
                            .await;
                    }
                    FlightState::Charge => {
                        let join_handle = async {
                            FlightComputer::set_state_wait(&f_cont_lock, FlightState::Charge).await;
                            println!("[INFO] State Change done! Hopefully export is also done!");
                        };
                        let c_cont_lock_clone = Arc::clone(&c_cont_lock);
                        let handle = tokio::spawn(async move {
                            let c_cont = c_cont_lock_clone.lock().await;
                            c_cont.create_snapshot_full().await.expect("[WARN] Export failed!");
                        });
                        tokio::join!(join_handle, handle);
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
}

fn handle_acquisition(
    f_cont_locked: &Arc<RwLock<FlightComputer>>,
    c_cont_locked: &Arc<Mutex<CameraController>>,
    end_time: DateTime<chrono::Utc>,
    img_dt: f32,
    angle: CameraAngle,
    ff_allowed: bool,
) -> (
    JoinHandle<()>,
    Arc<Notify>,
    Arc<Mutex<DateTime<chrono::Utc>>>,
) {
    let f_cont_lock_arc_clone = Arc::clone(f_cont_locked);
    let end_time_locked = Arc::new(Mutex::new(end_time));
    let end_time_cloned = Arc::clone(&end_time_locked);
    let c_cont_cloned = Arc::clone(c_cont_locked);
    let last_image_notify = Arc::new(Notify::new());
    let last_image_notify_cloned = Arc::clone(&last_image_notify);

    let handle = tokio::spawn(async move {
        CameraController::execute_acquisition_cycle(
            c_cont_cloned,
            f_cont_lock_arc_clone,
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
