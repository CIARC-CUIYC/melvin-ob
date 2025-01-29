#![allow(dead_code)]
#![warn(clippy::shadow_reuse, clippy::shadow_same, clippy::builtin_type_shadow)]

mod console_communication;
mod flight_control;
mod http_handler;
mod keychain;

use crate::flight_control::common::pinned_dt::PinnedTimeDelay;
use crate::flight_control::orbit::characteristics::OrbitCharacteristics;
use crate::flight_control::{
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
use crate::http_handler::ZonedObjective;
use crate::keychain::{Keychain, KeychainWithOrbit};
use crate::GlobalMode::ZonedObjectiveMode;
use crate::MappingModeEnd::{Join, Timestamp};
use chrono::{DateTime, TimeDelta};
use csv::Writer;
use std::collections::VecDeque;
use std::{env, fs::OpenOptions, sync::Arc};
use tokio::{sync::Notify, task::JoinHandle};

enum MappingModeEnd {
    Timestamp(DateTime<chrono::Utc>),
    Join(JoinHandle<()>),
}

enum GlobalMode {
    MappingMode,
    ZonedObjectiveMode(ZonedObjective),
}

const DT_MIN: TimeDelta = TimeDelta::seconds(5);
const DT_0: TimeDelta = TimeDelta::seconds(0);
const DT_0_STD: std::time::Duration = std::time::Duration::from_secs(0);
const DETUMBLE_TOL: TimeDelta = DT_MIN;

const STATIC_ORBIT_VEL: (f32, f32) = (6.4f32, 7.4f32);
pub const MIN_BATTERY_THRESHOLD: f32 = 10.0;
pub const MAX_BATTERY_THRESHOLD: f32 = 100.0;
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;

const POS_FILEPATH: &str = "pos.csv";
const BIN_FILEPATH: &str = "camera_controller_narrow.bin";

#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let base_url_var = env::var("DRS_BASE_URL");
    let base_url = base_url_var.as_ref().map_or("http://localhost:33000", |v| v.as_str());
    let (k, orbit_char) = {
        let res = init(base_url, BIN_FILEPATH).await;
        (Arc::new(res.0), res.1)
    };

    let sched = k.t_cont().sched_arc();

    let debug_objective = ZonedObjective::new(
        0,
        chrono::Utc::now(),
        chrono::Utc::now() + chrono::TimeDelta::hours(7),
        "Test Objective".to_string(),
        0,
        true,
        [4750, 5300, 5350, 5900],
        "narrow".to_string(),
        1.0,
        "Test Objective".to_string(),
        "test_objective.png".to_string(),
        false,
    );

    let mut objective_queue = VecDeque::new();
    objective_queue.push_back(debug_objective.clone());

    //schedule_zoned_objective_retrieval(Arc::clone(&k), orbit_char, debug_objective).await;
    let mut global_mode = GlobalMode::MappingMode;
    loop {
        schedule_undisturbed_orbit(Arc::clone(&k), orbit_char).await;

        let mut phases = 0;
        while let Some(task) = { (*sched).write().await.pop_front() } {
            phases += 1;
            let task_type = task.task_type();
            let mut due_time = task.dt().time_left();
            println!(
                "[INFO] Iteration {phases}: {task_type} in  {}s!",
                due_time.num_seconds()
            );

            let current_state = { k.f_cont().read().await.state() };
            if current_state == FlightState::Acquisition {
                match global_mode {
                    GlobalMode::MappingMode => {
                        if due_time > DT_MIN {
                            let k_clone = Arc::clone(&k);
                            execute_mapping(
                                k_clone,
                                Timestamp(chrono::Utc::now() + due_time),
                                orbit_char.img_dt(),
                                orbit_char.i_entry(),
                            )
                            .await;
                        }
                    }
                    GlobalMode::ZonedObjectiveMode(_) => {}
                }
                due_time = task.dt().time_left();
                FlightComputer::wait_for_duration(due_time.to_std().unwrap_or(DT_0_STD)).await;
            } else if current_state == FlightState::Charge && due_time > DT_0 {
                FlightComputer::wait_for_duration(due_time.to_std().unwrap_or(DT_0_STD)).await;
            } else {
                panic!("[FATAL] Illegal state ({current_state}) or too little time left for task ({due_time})!")
            }

            match task_type {
                BaseTask::TakeImage(_) => {
                    todo!()
                }
                BaseTask::ChangeVelocity(vel_change) => {
                    let burn = vel_change.burn();
                    for vel_change in burn.sequence_vel() {
                        let st = tokio::time::Instant::now();
                        let dt = std::time::Duration::from_secs(1);
                        FlightComputer::set_vel_wait(k.f_cont(), *vel_change).await;
                        let el = st.elapsed();
                        if el < dt {
                            tokio::time::sleep(dt).await;
                        }
                    }
                    let exp_pos = burn.sequence_pos().last().unwrap();
                    let current_pos = k.f_cont().read().await.current_pos();
                    let diff = *exp_pos - current_pos;
                    let detumble_time_delta = TimeDelta::seconds(burn.detumble_dt() as i64);
                    let detumble_dt = PinnedTimeDelay::new(detumble_time_delta - DETUMBLE_TOL);
                    println!("[INFO] Velocity change done! Expected position {exp_pos}, Actual Position {current_pos}, Diff {diff}");
                    let objective = objective_queue.pop_front().unwrap();
                    // TODO: here we shouldnt use objective.get_imaging_points but something already created,
                    // TODO: also the mode change to global mode should happen sometime else
                    let (vel, dev) = FlightComputer::evaluate_burn(
                        k.f_cont(),
                        burn,
                        objective.get_imaging_points()[0],
                    )
                    .await;
                    TaskController::calculate_orbit_correction_burn(vel, dev, detumble_dt);
                    global_mode = ZonedObjectiveMode(objective);
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
                        tokio::spawn(async move {
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

#[allow(clippy::cast_precision_loss)]
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
        f_cont_lock.write().await.reset().await;
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
            Join(schedule_join_handle),
            orbit_char.img_dt(),
            orbit_char.i_entry(),
        )
        .await;
    } else {
        schedule_join_handle.await.ok();
    }
}

async fn schedule_zoned_objective_retrieval(
    k_clone: Arc<KeychainWithOrbit>,
    orbit_char: OrbitCharacteristics,
    objective: ZonedObjective,
) {
    let schedule_join_handle = {
        let k_clone_clone = Arc::clone(&k_clone);
        tokio::spawn(async move {
            TaskController::schedule_zoned_objective(
                k_clone_clone.t_cont(),
                k_clone_clone.c_orbit(),
                k_clone_clone.f_cont(),
                orbit_char.i_entry(),
                objective,
            )
            .await;
        })
    };
    let current_state = k_clone.f_cont().read().await.state();

    if current_state == FlightState::Acquisition {
        let k_clone_clone = Arc::clone(&k_clone);
        execute_mapping(
            k_clone_clone,
            Join(schedule_join_handle),
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
            Timestamp(dt) => dt,
            Join(_) => chrono::Utc::now() + chrono::TimeDelta::seconds(10000),
        }
    };
    let k_clone_clone = Arc::clone(&k_clone);
    let acq_phase =
        start_periodic_imaging(k_clone_clone, end_t, img_dt, CONST_ANGLE, i_entry).await;
    if let Join(join_handle) = end {
        join_handle.await.ok();
        acq_phase.1.notify_one();
    }
    let ranges = acq_phase.0.await.ok().unwrap_or(Vec::new());
    let fixed_ranges = IndexedOrbitPosition::map_ranges(&ranges, i_entry.period() as isize);
    print!("[INFO] Marking done: {} - {}", ranges[0].0, ranges[0].1);
    if let Some(r) = ranges.get(1) {
        print!(" and {} - {}", r.0, r.1);
    }
    println!();
    let k_loc = Arc::clone(&k_clone);
    tokio::spawn(async move {
        let c_orbit_lock = k_loc.c_orbit();
        let mut c_orbit = c_orbit_lock.write().await;
        for (start, end) in &fixed_ranges {
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
) -> (JoinHandle<Vec<(isize, isize)>>, Arc<Notify>) {
    let f_cont_lock = Arc::clone(&k_clone.f_cont());
    let last_image_notify = Arc::new(Notify::new());
    let last_image_notify_cloned = Arc::clone(&last_image_notify);

    let i_start = i_shift.new_from_pos(f_cont_lock.read().await.current_pos());

    let handle = tokio::spawn(async move {
        k_clone
            .c_cont()
            .execute_acquisition_cycle(
                f_cont_lock,
                k_clone.con(),
                (end_time, last_image_notify_cloned),
                img_dt,
                angle,
                i_start.index(),
            )
            .await
    });
    (handle, last_image_notify)
}
