#![allow(dead_code, unused)]
#![warn(clippy::shadow_reuse, clippy::shadow_same, clippy::builtin_type_shadow)]

mod console_communication;
mod flight_control;
mod http_handler;

use crate::console_communication::console_messenger::ConsoleMessenger;
use crate::flight_control::{
    camera_controller::CameraController,
    camera_state::CameraAngle,
    common::vec2d::Vec2D,
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::{
        closed_orbit::{ClosedOrbit, OrbitUsabilityError},
        index::IndexedOrbitPosition,
        orbit_base::OrbitBase,
    },
    task::{base_task::BaseTask, task_controller::TaskController},
};
use crate::http_handler::{
    http_client::HTTPClient,
    http_request::{observation_get::ObservationRequest, request_common::NoBodyHTTPRequestType},
};
use chrono::DateTime;
use csv::Writer;
use std::{env, fs::OpenOptions, io::Write, sync::Arc};
use tokio::{
    sync::{Mutex, Notify, RwLock},
    task::JoinHandle,
};

const DT_0: chrono::TimeDelta = chrono::TimeDelta::seconds(0);

const STATIC_ORBIT_VEL: (f32, f32) = (6.4f32, 7.4f32);
pub const MIN_BATTERY_THRESHOLD: f32 = 10.0;
pub const MAX_BATTERY_THRESHOLD: f32 = 100.0;
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;

const POS_FILEPATH: &str = "pos.csv";
const BIN_FILEPATH: &str = "camera_controller_narrow.bin";

#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::too_many_lines,
    clippy::cast_precision_loss
)]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let base_url_var = env::var("DRS_BASE_URL");
    let base_url = base_url_var.as_ref().map_or("http://localhost:33000", |v| v.as_str());
    let client = Arc::new(HTTPClient::new(base_url));

    let t_cont = TaskController::new();
    let c_cont = Arc::new(
        CameraController::from_file(BIN_FILEPATH, client.clone()).await.unwrap_or_else(|e| {
            println!("[WARN] Failed to read from binary file: {e}");
            CameraController::new(client.clone())
        }),
    );
    let f_cont_lock = Arc::new(RwLock::new(FlightComputer::new(Arc::clone(&client)).await));
    let console_messenger = Arc::new(ConsoleMessenger::start(c_cont.clone()));
    let f_cont_lock_clone = Arc::clone(&f_cont_lock);

    // spawn logging task
    tokio::spawn(async move {
        let file =
            OpenOptions::new().write(true).create(true).truncate(true).open(POS_FILEPATH).unwrap();
        let mut wrt = Writer::from_writer(file);
        let mut first = true; // DEBUG ARTIFACT
        let mut last_pos: Vec2D<f32> = Vec2D::new(-100.0, -100.0);
        let mut last_timestamp = chrono::Utc::now();
        loop {
            {
                let mut f_cont = f_cont_lock_clone.write().await;
                (*f_cont).update_observation().await;
                if first && f_cont.current_vel().eq(&STATIC_ORBIT_VEL.into()) {
                    last_pos = f_cont.current_pos();
                    drop(f_cont);
                    last_timestamp = chrono::Utc::now();
                    first = false;
                } else if !first {
                    let current_pos = f_cont.current_pos();
                    let current_state_str = f_cont.state().to_string();
                    drop(f_cont);
                    let dt = chrono::Utc::now() - last_timestamp;
                    last_timestamp = chrono::Utc::now();
                    let mut expected_pos = last_pos
                        + <(f32, f32) as Into<Vec2D<f32>>>::into(STATIC_ORBIT_VEL)
                            * dt.num_milliseconds() as f32
                            / 1000.0;
                    expected_pos = expected_pos.wrap_around_map();
                    let diff = current_pos - expected_pos;
                    wrt.write_record(&[
                        current_pos.x().to_string(),
                        current_pos.y().to_string(),
                        expected_pos.x().to_string(),
                        expected_pos.y().to_string(),
                        diff.x().to_string(),
                        diff.y().to_string(),
                        current_state_str,
                    ])
                    .unwrap();
                    last_pos = expected_pos;
                }
            };
            tokio::time::sleep(std::time::Duration::from_millis(400)).await;
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
        + chrono::TimeDelta::seconds(c_orbit.period().0 as i64);
    let orbit_full_period = c_orbit.period().0 as usize;
    // TODO: this must be created dynamically in the loop at some time
    let i_entry = IndexedOrbitPosition::new(0, orbit_full_period, {
        f_cont_lock.read().await.current_pos()
    });

    let c_orbit_lock = Arc::new(RwLock::new(c_orbit));
    let t_cont_arc = Arc::new(t_cont);
    let sched = t_cont_arc.sched_arc();
    // orbit
    loop {
        let f_cont_rw_clone = Arc::clone(&f_cont_lock);
        let c_orbit_lock_clone = Arc::clone(&c_orbit_lock);
        let t_cont_arc_clone = Arc::clone(&t_cont_arc);
        let schedule_join_handle = tokio::spawn(async move {
            TaskController::schedule_optimal_orbit(
                t_cont_arc_clone,
                c_orbit_lock_clone,
                f_cont_rw_clone,
                i_entry,
            )
            .await;
        });
        let current_state = f_cont_lock.read().await.state();

        let acq_phase = if current_state == FlightState::Acquisition {
            let end_time = chrono::Utc::now() + chrono::TimeDelta::seconds(10000);
            Some(
                handle_acquisition(
                    &f_cont_lock,
                    console_messenger.clone(),
                    c_cont.clone(),
                    end_time,
                    img_dt,
                    CONST_ANGLE,
                    i_entry,
                )
                .await,
            )
        } else {
            None
        };

        schedule_join_handle.await.ok();
        if let Some(phase) = acq_phase {
            phase.1.notify_one();
            phase.0.await.ok();
            let ranges = phase.2.get_ranges_to_now();
            let c_orbit_lock_clone = Arc::clone(&c_orbit_lock);
            print!("[INFO] Marking done: {} - {}", ranges[0].0, ranges[0].1);
            if let Some(r) = ranges.get(1) {
                print!(" and {} - {}", r.0, r.1);
            }
            println!();
            tokio::spawn(async move {
                let mut c_orbit = c_orbit_lock_clone.write().await;
                for (start, end) in &ranges {
                    c_orbit.mark_done(*start, *end);
                }
            });
        }

        let mut phases = 0;

        while let Some(task) = { (*sched).write().await.pop_front() } {
            phases += 1;
            let task_type = task.task_type();
            let due_time = task.dt().time_left();
            println!(
                "[INFO] Iteration {phases}: {task_type} in  {}s!",
                due_time.num_seconds()
            );

            let current_state = { f_cont_lock.read().await.state() };
            if current_state == FlightState::Acquisition && due_time > DT_0 {
                let acq_phase = handle_acquisition(
                    &f_cont_lock,
                    console_messenger.clone(),
                    c_cont.clone(),
                    task.dt().get_end(),
                    img_dt,
                    CONST_ANGLE,
                    i_entry,
                )
                .await;
                acq_phase.0.await.ok();
                let ranges = acq_phase.2.get_ranges_to_now();
                let c_orbit_lock_clone = Arc::clone(&c_orbit_lock);
                print!("[INFO] Marking done: {} - {}", ranges[0].0, ranges[0].1);
                if let Some(r) = ranges.get(1) {
                    print!(" and {} - {}", r.0, r.1);
                }
                println!();
                tokio::spawn(async move {
                    let mut c_orbit = c_orbit_lock_clone.write().await;
                    for (start, end) in &ranges {
                        c_orbit.mark_done(*start, *end);
                    }
                });
            } else if current_state == FlightState::Charge && due_time > DT_0 {
                let task_due = task.dt().time_left();
                FlightComputer::wait_for_duration(task_due.to_std().unwrap()).await;
            } else {
                panic!("[FATAL] Illegal state ({current_state}) or too little time left for task ({due_time})!")
            }
            match task_type {
                BaseTask::TakeImage(_) => {
                    todo!()
                }
                BaseTask::ChangeVelocity(_) => {
                    todo!()
                }
                BaseTask::SwitchState(switch) => match switch.target_state() {
                    FlightState::Acquisition => {
                        FlightComputer::set_state_wait(&f_cont_lock, FlightState::Acquisition)
                            .await;
                    }
                    FlightState::Charge => {
                        let join_handle = async {
                            FlightComputer::set_state_wait(&f_cont_lock, FlightState::Charge).await;
                        };
                        let c_cont_local = c_cont.clone();
                        let handle = tokio::spawn(async move {
                            c_cont_local
                                .create_snapshot_full()
                                .await
                                .expect("[WARN] Export failed!");
                        });
                        join_handle.await;
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
    // drop(console_messenger);
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
async fn handle_acquisition(
    f_cont_locked: &Arc<RwLock<FlightComputer>>,
    console_messenger: Arc<ConsoleMessenger>,
    c_cont: Arc<CameraController>,
    end_time: DateTime<chrono::Utc>,
    img_dt: f32,
    angle: CameraAngle,
    i_shift: IndexedOrbitPosition,
) -> (JoinHandle<()>, Arc<Notify>, IndexedOrbitPosition) {
    let f_cont_lock_arc_clone = Arc::clone(f_cont_locked);
    let last_image_notify = Arc::new(Notify::new());
    let last_image_notify_cloned = Arc::clone(&last_image_notify);

    let i_start = i_shift.new_from_pos({ f_cont_lock_arc_clone.read().await.current_pos() });

    let handle = tokio::spawn(async move {
        c_cont
            .execute_acquisition_cycle(
                f_cont_lock_arc_clone,
                &console_messenger,
                end_time,
                last_image_notify_cloned,
                img_dt,
                angle,
            )
            .await;
    });
    (handle, last_image_notify, i_start)
}
