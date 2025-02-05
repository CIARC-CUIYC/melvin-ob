#![allow(dead_code)]
#![warn(clippy::shadow_reuse, clippy::shadow_same, clippy::builtin_type_shadow)]

mod console_communication;
mod flight_control;
mod http_handler;
mod keychain;

use crate::flight_control::supervisor::Supervisor;
use crate::flight_control::{
    camera_state::CameraAngle,
    common::pinned_dt::PinnedTimeDelay,
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::{
        ClosedOrbit, IndexedOrbitPosition, OrbitBase, OrbitCharacteristics, OrbitUsabilityError,
    },
    task::{base_task::BaseTask, TaskController},
};
use crate::http_handler::ZonedObjective;
use crate::keychain::{Keychain, KeychainWithOrbit};
use crate::MappingModeEnd::{Join, Timestamp};
use chrono::{DateTime, TimeDelta};
use fixed::types::I32F32;
use std::future::Future;
use std::pin::Pin;
use std::{
    collections::VecDeque,
    {env, sync::Arc},
};
use strum_macros::Display;
use tokio::sync::oneshot;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

enum MappingModeEnd {
    Timestamp(DateTime<chrono::Utc>),
    Join(JoinHandle<()>),
}

#[derive(Debug)]
enum PeriodicImagingEndSignal {
    KillNow,
    KillLastImage,
}

#[derive(Display)]
enum GlobalMode {
    MappingMode,
    ZonedObjectiveMode(ZonedObjective),
}

const DT_MIN: TimeDelta = TimeDelta::seconds(5);
const DT_0: TimeDelta = TimeDelta::seconds(0);
const DT_0_STD: std::time::Duration = std::time::Duration::from_secs(0);
const DETUMBLE_TOL: TimeDelta = DT_MIN;

const STATIC_ORBIT_VEL: (I32F32, I32F32) = (I32F32::lit("6.4"), I32F32::lit("7.4"));
pub const MIN_BATTERY_THRESHOLD: I32F32 = I32F32::lit("10.0");
pub const MAX_BATTERY_THRESHOLD: I32F32 = I32F32::lit("100.0");
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;

#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let base_url_var = env::var("DRS_BASE_URL");
    let base_url = base_url_var.as_ref().map_or("http://localhost:33000", |v| v.as_str());
    let (k, mut orbit_char, supervisor) = {
        let res = init(base_url).await;
        (Arc::new(res.0), res.1, res.2)
    };

    let sched = k.t_cont().sched_arc();

    let debug_objective = ZonedObjective::new(
        0,
        chrono::Utc::now(),
        chrono::Utc::now() + TimeDelta::hours(7),
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
    let safe_event_notify = supervisor.safe_mode_notify();

    'outer: loop {
        let mut phases = 0;
        println!("[INFO] Starting new phase in {global_mode}!");
        let cancel_task = CancellationToken::new();
        let fut = match global_mode {
            GlobalMode::MappingMode => {
                schedule_undisturbed_orbit(Arc::clone(&k), orbit_char, cancel_task.clone())
            }
            GlobalMode::ZonedObjectiveMode(_) => {
                todo!()
            }
        };

        tokio::select!(
            () = fut => {
                k.con().send_tasklist().await;
            },
            () = safe_event_notify.notified() => {
                cancel_task.cancel();
                FlightComputer::escape_safe(k.f_cont()).await;
                continue 'outer;
            }
        );

        while let Some(task) = { (*sched).write().await.pop_front() } {
            phases += 1;
            let task_type = task.task_type();
            let due_time = task.dt().time_left();
            println!(
                "[INFO] Iteration {phases}: {task_type} in  {}s!",
                due_time.num_seconds()
            );

            let current_state = { k.f_cont().read().await.state() };
            let cancel_task = CancellationToken::new();
            let task_fut: Pin<Box<dyn Future<Output = ()>>> = match current_state {
                FlightState::Acquisition => match global_mode {
                    GlobalMode::MappingMode => {
                        if due_time > DT_MIN {
                            let k_clone = Arc::clone(&k);
                            Box::pin(execute_mapping(
                                k_clone,
                                Timestamp(chrono::Utc::now() + due_time),
                                orbit_char.img_dt(),
                                orbit_char.i_entry(),
                                cancel_task.clone(),
                            ))
                        } else {
                            println!("This never happens, right?");
                            Box::pin(tokio::time::sleep(due_time.to_std().unwrap_or(DT_0_STD)))
                        }
                    }
                    GlobalMode::ZonedObjectiveMode(_) => {
                        todo!()
                    }
                },
                FlightState::Charge => Box::pin(FlightComputer::wait_for_duration(
                    due_time.to_std().unwrap_or(DT_0_STD),
                )),
                FlightState::Comms => {
                    todo!()
                }
                _ => {
                    panic!("[FATAL] Illegal state ({current_state})!")
                }
            };

            tokio::select! {
                () = task_fut => {},
                () = safe_event_notify.notified() => {
                        cancel_task.cancel();
                        FlightComputer::escape_safe(k.f_cont()).await;
                        // TODO: here it should be checked if we were in objective retrieval
                        continue 'outer;
                }
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
                    global_mode = GlobalMode::ZonedObjectiveMode(objective);
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
                                .create_full_snapshot()
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
        }
        phases += 1;
        orbit_char.finish(orbit_char.i_entry().new_from_pos(k.f_cont().read().await.current_pos()));
    }
    // drop(console_messenger);
}

#[allow(clippy::cast_precision_loss)]
async fn init(url: &str) -> (KeychainWithOrbit, OrbitCharacteristics, Arc<Supervisor>) {
    let init_k = Keychain::new(url).await;
    let init_k_f_cont_clone = init_k.f_cont();
    let supervisor = Arc::new(Supervisor::new(init_k_f_cont_clone));
    let supervisor_clone = Arc::clone(&supervisor);
    tokio::spawn(async move {
        supervisor_clone.run().await;
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

    supervisor.reset_pos_monitor().notify_one();

    let orbit_char = OrbitCharacteristics::new(&c_orbit, &init_k.f_cont()).await;
    (
        KeychainWithOrbit::new(init_k, c_orbit),
        orbit_char,
        supervisor,
    )
}

async fn schedule_undisturbed_orbit(
    k_clone: Arc<KeychainWithOrbit>,
    orbit_char: OrbitCharacteristics,
    cancel_token: CancellationToken,
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
            cancel_token,
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
    cancel_token: CancellationToken,
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
            cancel_token,
        )
        .await;
    } else {
        schedule_join_handle.await.ok();
    }
}

#[allow(clippy::cast_possible_wrap)]
async fn execute_mapping(
    k_clone: Arc<KeychainWithOrbit>,
    end: MappingModeEnd,
    img_dt: I32F32,
    i_entry: IndexedOrbitPosition,
    cancel_token: CancellationToken,
) {
    let end_t = {
        match end {
            Timestamp(dt) => dt,
            Join(_) => chrono::Utc::now() + TimeDelta::seconds(10000),
        }
    };
    let k_clone_clone = Arc::clone(&k_clone);
    let acq_phase =
        start_periodic_imaging(k_clone_clone, end_t, img_dt, CONST_ANGLE, i_entry).await;

    let ranges = {
        if let Join(join_handle) = end {
            let ((), res) = tokio::join!(
                async move {
                    tokio::select! {
                        () = cancel_token.cancelled() => {
                            let sig = PeriodicImagingEndSignal::KillNow;
                            acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        },
                        _ = join_handle => {
                            let sig = PeriodicImagingEndSignal::KillLastImage;
                            acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        }
                    }
                },
                async move { acq_phase.0.await.ok().unwrap_or(Vec::new()) }
            );
            res
        } else {
            tokio::select! {
                () = cancel_token.cancelled() => {
                    let sig = PeriodicImagingEndSignal::KillNow;
                    acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                    Vec::new()
                }
                res = acq_phase.0 => {
                    res.ok().unwrap_or(Vec::new())
                }
            }
        }
    };

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
    img_dt: I32F32,
    angle: CameraAngle,
    i_shift: IndexedOrbitPosition,
) -> (
    JoinHandle<Vec<(isize, isize)>>,
    oneshot::Sender<PeriodicImagingEndSignal>,
) {
    let f_cont_lock = Arc::clone(&k_clone.f_cont());
    let (tx, rx) = oneshot::channel();

    let i_start = i_shift.new_from_pos(f_cont_lock.read().await.current_pos());

    let handle = tokio::spawn(async move {
        k_clone
            .c_cont()
            .execute_acquisition_cycle(
                f_cont_lock,
                k_clone.con(),
                (end_time, rx),
                img_dt,
                angle,
                i_start.index(),
            )
            .await
    });
    (handle, tx)
}
