#![allow(dead_code, unused)]
#![warn(clippy::shadow_reuse, clippy::shadow_same, clippy::builtin_type_shadow)]

mod console_communication;
mod flight_control;
mod http_handler;
mod keychain;

use crate::console_communication::console_messenger::ConsoleMessenger;
use crate::flight_control::orbit::characteristics::OrbitCharacteristics;
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
use crate::keychain::{Keychain, KeychainWithOrbit};
use crate::MappingModeEnd::{DT, JOIN};
use chrono::DateTime;
use csv::Writer;
use std::{env, fs::OpenOptions, io::Write, sync::Arc};
use tokio::{
    sync::{Mutex, Notify, RwLock},
    task::JoinHandle,
};

enum MappingModeEnd {
    DT(DateTime<chrono::Utc>),
    JOIN(JoinHandle<()>),
}

const DT_0: chrono::TimeDelta = chrono::TimeDelta::seconds(0);

const STATIC_ORBIT_VEL: (f32, f32) = (6.4f32, 7.4f32);
pub const MIN_BATTERY_THRESHOLD: f32 = 10.0;
pub const MAX_BATTERY_THRESHOLD: f32 = 100.0;
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;

const POS_FILEPATH: &str = "pos.csv";
const BIN_FILEPATH: &str = "camera_controller_narrow.bin";

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_precision_loss)]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let base_url_var = env::var("DRS_BASE_URL");
    let base_url = base_url_var.as_ref().map_or("http://localhost:33000", |v| v.as_str());
    let (k, orbit_char) = {
        let res = init(base_url, BIN_FILEPATH).await;
        (Arc::new(res.0), res.1)
    };

    let sched = k.t_cont().sched_arc();

    loop {
        schedule_undisturbed_orbit(Arc::clone(&k), orbit_char).await;
        let mut phases = 0;
        while let Some(task) = { (*sched).write().await.pop_front() } {
            phases += 1;
            let task_type = task.task_type();
            let due_time = task.dt().time_left();
            println!(
                "[INFO] Iteration {phases}: {task_type} in  {}s!",
                due_time.num_seconds()
            );

            let current_state = { k.f_cont().read().await.state() };
            if current_state == FlightState::Acquisition && due_time > DT_0 {
                let k_clone = Arc::clone(&k);
                execute_mapping(
                    k_clone,
                    DT(chrono::Utc::now() + due_time),
                    orbit_char.img_dt(),
                    orbit_char.i_entry(),
                )
                .await;
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
                        FlightComputer::set_state_wait(k.f_cont(), FlightState::Acquisition).await;
                    }
                    FlightState::Charge => {
                        let join_handle = async {
                            FlightComputer::set_state_wait(k.f_cont(), FlightState::Charge).await;
                        };
                        let k_clone = Arc::clone(&k);
                        let handle = tokio::spawn(async move {
                            k_clone
                                .c_cont()
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

async fn init(url: &str, c_cont_file: &str) -> (KeychainWithOrbit, OrbitCharacteristics) {
    let init_k = Keychain::new(url, c_cont_file).await;
    let init_k_f_cont_clone = init_k.f_cont();
    tokio::spawn(async move {
        let file =
            OpenOptions::new().write(true).create(true).truncate(true).open(POS_FILEPATH).unwrap();
        let mut wrt = Writer::from_writer(file);
        let mut first = true; // DEBUG ARTIFACT
        let mut last_pos: Vec2D<f32> = Vec2D::new(-100.0, -100.0);
        let mut last_timestamp = chrono::Utc::now();
        loop {
            {
                let mut f_cont = init_k_f_cont_clone.write().await;
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
        let f_cont_lock = init_k.f_cont();
        FlightComputer::set_state_wait(init_k.f_cont(), FlightState::Acquisition).await;
        FlightComputer::set_vel_wait(init_k.f_cont(), STATIC_ORBIT_VEL.into()).await;
        FlightComputer::set_angle_wait(init_k.f_cont(), CONST_ANGLE).await;
        let f_cont = f_cont_lock.read().await;
        ClosedOrbit::new(OrbitBase::new(&f_cont), CameraAngle::Wide).unwrap_or_else(|e| match e {
            OrbitUsabilityError::OrbitNotClosed => panic!("[FATAL] Static orbit is not closed"),
            OrbitUsabilityError::OrbitNotEnoughOverlap => {
                panic!("[FATAL] Static orbit is not overlapping enough")
            }
        })
    };

    let orbit_char = OrbitCharacteristics::new(&c_orbit, &init_k.f_cont()).await;
    (KeychainWithOrbit::new(init_k, c_orbit), orbit_char)
}

async fn schedule_undisturbed_orbit(
    k_clone: Arc<KeychainWithOrbit>,
    orbit_char: OrbitCharacteristics,
) {
    let schedule_join_handle = {
        let k_clone_clone = Arc::clone(&k_clone);
        tokio::spawn(async move {
            TaskController::schedule_optimal_orbit(
                k_clone_clone.t_cont(),
                k_clone_clone.c_orbit(),
                k_clone_clone.f_cont(),
                orbit_char.i_entry(),
            )
            .await;
        })
    };
    let current_state = k_clone.f_cont().read().await.state();

    if current_state == FlightState::Acquisition {
        let k_clone_clone = Arc::clone(&k_clone);
        execute_mapping(
            k_clone_clone,
            JOIN(schedule_join_handle),
            orbit_char.img_dt(),
            orbit_char.i_entry(),
        )
        .await;
    } else {
        schedule_join_handle.await.ok();
    }
}

async fn execute_mapping(
    k_clone: Arc<KeychainWithOrbit>,
    end: MappingModeEnd,
    img_dt: f32,
    i_entry: IndexedOrbitPosition,
) {
    let end_t = {
        match end {
            DT(dt) => dt,
            JOIN(_) => chrono::Utc::now() + chrono::TimeDelta::seconds(10000),
        }
    };
    let k_clone_clone = Arc::clone(&k_clone);
    let acq_phase =
        start_periodic_imaging(k_clone_clone, end_t, img_dt, CONST_ANGLE, i_entry).await;
    if let JOIN(join_handle) = end {
        join_handle.await.ok();
        acq_phase.1.notify_one();
    }
    acq_phase.0.await.ok();
    let ranges = acq_phase.2.get_ranges_to_now();
    print!("[INFO] Marking done: {} - {}", ranges[0].0, ranges[0].1);
    if let Some(r) = ranges.get(1) {
        print!(" and {} - {}", r.0, r.1);
    }
    println!();
    let k_loc = Arc::clone(&k_clone);
    tokio::spawn(async move {
        let c_orbit_lock = k_loc.c_orbit();
        let mut c_orbit = c_orbit_lock.write().await;
        for (start, end) in &ranges {
            c_orbit.mark_done(*start, *end);
        }
    });
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
async fn start_periodic_imaging(
    k_clone: Arc<KeychainWithOrbit>,
    end_time: DateTime<chrono::Utc>,
    img_dt: f32,
    angle: CameraAngle,
    i_shift: IndexedOrbitPosition,
) -> (JoinHandle<()>, Arc<Notify>, IndexedOrbitPosition) {
    let f_cont_lock = Arc::clone(&k_clone.f_cont());
    let last_image_notify = Arc::new(Notify::new());
    let last_image_notify_cloned = Arc::clone(&last_image_notify);

    let i_start = i_shift.new_from_pos({ f_cont_lock.read().await.current_pos() });

    let handle = tokio::spawn(async move {
        k_clone
            .c_cont()
            .execute_acquisition_cycle(
                f_cont_lock,
                k_clone.con(),
                end_time,
                last_image_notify_cloned,
                img_dt,
                angle,
            )
            .await;
    });
    (handle, last_image_notify, i_start)
}
