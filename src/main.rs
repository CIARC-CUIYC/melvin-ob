#![allow(dead_code)]
#![warn(clippy::shadow_reuse, clippy::shadow_same, clippy::builtin_type_shadow)]

mod console_communication;
mod flight_control;
mod global_mode;
mod http_handler;
mod keychain;

use crate::flight_control::objective::beacon_objective::BeaconObjective;
use crate::flight_control::objective::objective_base::ObjectiveBase;
use crate::flight_control::objective::objective_type::ObjectiveType;
use crate::flight_control::objective::secret_img_objective::SecretImgObjective;
use crate::flight_control::supervisor::Supervisor;
use crate::flight_control::task::vel_change_task::{
    VelocityChangeTask, VelocityChangeTaskRationale,
};
use crate::flight_control::{
    camera_state::CameraAngle,
    common::pinned_dt::PinnedTimeDelay,
    flight_computer::FlightComputer,
    flight_state::FlightState,
    objective::known_img_objective::KnownImgObjective,
    orbit::{
        ClosedOrbit, IndexedOrbitPosition, OrbitBase, OrbitCharacteristics, OrbitUsabilityError,
    },
    task::{base_task::BaseTask, TaskController},
};
use crate::keychain::{Keychain, KeychainWithOrbit};
use crate::MappingModeEnd::{Join, Timestamp};
use chrono::{DateTime, TimeDelta};
use fixed::types::I32F32;
use global_mode::GlobalMode;
use std::collections::BinaryHeap;
use std::future::Future;
use std::pin::Pin;
use std::{
    collections::VecDeque,
    {env, sync::Arc},
};
use tokio::sync::mpsc::Receiver;
use tokio::{sync::oneshot, task::JoinHandle};
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

const SAFE_MODE_RATIONALE: &str = "SAFE mode event!";
const NEW_ZO_RATIONALE: &str = "newly discovered ZO!";
const NEW_BO_RATIONALE: &str = "newly discovered BO!";
const TASKS_DONE_RATIONALE: &str = "tasks list done!";

const DT_MIN: TimeDelta = TimeDelta::seconds(5);
const DT_0: TimeDelta = TimeDelta::seconds(0);
const DT_0_STD: std::time::Duration = std::time::Duration::from_secs(0);
const DETUMBLE_TOL: TimeDelta = DT_MIN;

const STATIC_ORBIT_VEL: (I32F32, I32F32) = (I32F32::lit("6.40"), I32F32::lit("7.40"));
pub const MIN_BATTERY_THRESHOLD: I32F32 = I32F32::lit("10.00");
pub const MAX_BATTERY_THRESHOLD: I32F32 = I32F32::lit("100.00");
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
    let (k, mut orbit_char, supervisor, mut obj_monitor) = {
        let res = init(base_url).await;
        (Arc::new(res.0), res.1, res.2, res.3)
    };
    let sched = k.t_cont().sched_arc();

    let debug_objective = KnownImgObjective::new(
        0,
        "Test Objective".to_string(),
        chrono::Utc::now(),
        chrono::Utc::now() + TimeDelta::hours(7),
        [4750, 5300, 5350, 5900],
        "narrow".into(),
        100.0,
    );

    // TODO: debug artifact
    // objective_queue.push_back(ObjectiveType::KnownImage(debug_objective.clone()));
    let mut k_img_buffer: BinaryHeap<KnownImgObjective> = BinaryHeap::new();
    let mut beacon_buffer: BinaryHeap<BeaconObjective> = BinaryHeap::new();
    let mut s_img_buffer: VecDeque<SecretImgObjective> = VecDeque::new();

    let mut global_mode = GlobalMode::MappingMode;
    let safe_event_notify = supervisor.safe_mode_monitor();

    recv_all_obj(
        &mut obj_monitor,
        &mut k_img_buffer,
        &mut beacon_buffer,
        &mut s_img_buffer,
    );

    let mut phases = 0;
    'outer: loop {
        println!("[INFO] Starting phase {phases} in {global_mode}!");
        let cancel_task = CancellationToken::new();
        if k_img_buffer.is_empty() && beacon_buffer.is_empty() {
            global_mode = GlobalMode::MappingMode;
        } else {
            // TODO: handle multiple tasks at the same time
            // TODO: handle tasks at all with some kind of "StateResolver"
            match &global_mode {
                GlobalMode::MappingMode => {}
                GlobalMode::BeaconObjectivePassiveScanningMode(obj) => {}
                GlobalMode::BeaconObjectiveActiveScanningMode(_) => {}
                GlobalMode::ZonedObjectivePrepMode(obj) => {}
                GlobalMode::ZonedObjectiveRetrievalMode(_) => {}
                GlobalMode::ZonedObjectiveReturnMode => {}
            }
        }

        let fut = match global_mode {
            GlobalMode::MappingMode => {
                schedule_undisturbed_orbit(Arc::clone(&k), orbit_char, cancel_task.clone())
            }
            GlobalMode::ZonedObjectivePrepMode(obj) => {
                schedule_zoned_objective_retrieval(
                    Arc::clone(&k),
                    orbit_char,
                    cancel_task.clone(),
                    obj.clone(),
                )
                .await;
                todo!()
            }
            GlobalMode::BeaconObjectivePassiveScanningMode(obj) => {
                todo!()
            }
            _ => panic!("[FATAL] Unexpected global mode!"),
        };

        let task = tokio::spawn(fut);
        tokio::select!(
            _ = task => {
                k.con().send_tasklist().await;
            },
            () = safe_event_notify.notified() => {
                cancel_task.cancel();
                FlightComputer::escape_safe(k.f_cont()).await;
                orbit_char.finish(
                    orbit_char.i_entry().new_from_pos(k.f_cont().read().await.current_pos()),
                    SAFE_MODE_RATIONALE
                );
                phases += 1;
                continue 'outer;
            }
        );

        while let Some(task) = {
            let mut sched_lock = sched.write().await;
            let t = sched_lock.pop_front();
            drop(sched_lock);
            t
        } {
            phases += 1;
            let task_type = task.task_type();
            let due_time = task.dt().time_left();
            println!(
                "[INFO] Iteration {phases}: {task_type} in  {}s!",
                due_time.num_seconds()
            );

            let current_state = { k.f_cont().read().await.state() };
            let cancel_task = CancellationToken::new();

            let task_fut: Pin<Box<dyn Future<Output = ()> + Send>> = match current_state {
                FlightState::Acquisition => {
                    if due_time > DT_MIN && global_mode.should_map() {
                        let k_clone = Arc::clone(&k);
                        Box::pin(execute_mapping(
                            k_clone,
                            Timestamp(chrono::Utc::now() + due_time),
                            orbit_char.img_dt(),
                            orbit_char.i_entry(),
                            cancel_task.clone(),
                        ))
                    } else {
                        println!("[LOG] This acquisition mode is not eligible for mapping. Waiting for {} s", due_time.num_seconds());
                        Box::pin(tokio::time::sleep(due_time.to_std().unwrap_or(DT_0_STD)))
                    }
                }
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
            let task_handle = tokio::spawn(task_fut);
            tokio::select! {
                _ = task_handle => {},
                () = safe_event_notify.notified() => {
                        cancel_task.cancel();
                        FlightComputer::escape_safe(k.f_cont()).await;
                        k.t_cont().clear_schedule().await;
                        // TODO: here it should be checked if we were in objective retrieval
                        // TODO: the next line should only be executed if in mapping mode like this
                        orbit_char.finish(
                            orbit_char.i_entry().new_from_pos(k.f_cont().read().await.current_pos()),
                            SAFE_MODE_RATIONALE);
                        phases += 1;
                        continue 'outer;
                }
            }

            match task_type {
                BaseTask::TakeImage(_) => {
                    todo!()
                }

                BaseTask::ChangeVelocity(vel_change) => match vel_change.rationale() {
                    VelocityChangeTaskRationale::CorrectionalChange => {
                        todo!();
                        FlightComputer::execute_burn(k.f_cont(), vel_change.burn()).await;
                    }
                    VelocityChangeTaskRationale::OrbitEscapeChange => {
                        global_mode = handle_orbit_escape(global_mode, vel_change, &k).await;
                    }
                    VelocityChangeTaskRationale::OrbitEnterChange => {
                        todo!();
                        FlightComputer::execute_burn(k.f_cont(), vel_change.burn()).await;
                    }
                },

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
                        let new_obj = recv_all_obj(
                            &mut obj_monitor,
                            &mut k_img_buffer,
                            &mut beacon_buffer,
                            &mut s_img_buffer,
                        );
                        join_handle.await;
                        if new_obj.0 != 0 || new_obj.1 != 0 {
                            println!("[INFO] New objective detected, exiting phase {phases}!");
                            continue 'outer;
                        }
                    }
                    FlightState::Comms => {}
                    _ => {
                        panic!("[FATAL] Illegal target state!")
                    }
                },
            }
        }
        orbit_char.finish(
            orbit_char.i_entry().new_from_pos(k.f_cont().read().await.current_pos()),
            TASKS_DONE_RATIONALE,
        );
        phases += 1;
    }
    // drop(console_messenger);
}

#[allow(clippy::cast_precision_loss)]
async fn init(
    url: &str,
) -> (
    KeychainWithOrbit,
    OrbitCharacteristics,
    Arc<Supervisor>,
    Receiver<ObjectiveBase>,
) {
    let init_k = Keychain::new(url).await;
    init_k.f_cont().write().await.reset().await;
    let init_k_f_cont_clone = init_k.f_cont();
    let (supervisor, obj_rx) = {
        let (sv, rx) = Supervisor::new(init_k_f_cont_clone);
        (Arc::new(sv), rx)
    };
    let supervisor_clone = Arc::clone(&supervisor);
    tokio::spawn(async move {
        supervisor_clone.run().await;
    });

    tokio::time::sleep(DT_MIN.to_std().unwrap()).await;

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

    supervisor.reset_pos_monitor().notify_one();

    let orbit_char = OrbitCharacteristics::new(&c_orbit, &init_k.f_cont()).await;
    (
        KeychainWithOrbit::new(init_k, c_orbit),
        orbit_char,
        supervisor,
        obj_rx,
    )
}

fn recv_all_obj(
    obj_monitor: &mut Receiver<ObjectiveBase>,
    k_img_buffer: &mut BinaryHeap<KnownImgObjective>,
    beacon_buffer: &mut BinaryHeap<BeaconObjective>,
    s_img_buffer: &mut VecDeque<SecretImgObjective>,
) -> (usize, usize) {
    let mut k_img_count = 0;
    let mut beacon_count = 0;
    while let Ok(obj) = obj_monitor.try_recv() {
        let id = obj.id();
        let name = obj.name().to_string();
        let start = obj.start();
        let end = obj.end();

        match obj.obj_type() {
            ObjectiveType::Beacon { attempts_made } => {
                /*beacon_buffer.push(BeaconObjective::new(id, name, start, end));
                beacon_count += 1;*/
            }
            ObjectiveType::SecretImage {
                optic_required,
                coverage_required,
            } => {
                /*s_img_buffer.push_front(SecretImgObjective::new(
                    id,
                    name,
                    start,
                    end,
                    *optic_required,
                    *coverage_required,
                ));*/
            }
            ObjectiveType::KnownImage {
                zone,
                optic_required,
                coverage_required,
            } => {
                /*k_img_buffer.push(KnownImgObjective::new(
                    id,
                    name,
                    start,
                    end,
                    *zone,
                    *optic_required,
                    *coverage_required,
                ));
                k_img_count += 1;*/
            }
        }
        println!("[INFO] Found new objective: {}!", obj.obj_type());
    }
    (k_img_count, beacon_count)
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
    cancel_token: CancellationToken,
    objective: KnownImgObjective,
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
            let img_fut = acq_phase.0;
            tokio::pin!(img_fut);
            tokio::select! {
                () = cancel_token.cancelled() => {
                    let sig = PeriodicImagingEndSignal::KillNow;
                    acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                    img_fut.await.ok().unwrap_or(Vec::new())
                }
                res = &mut img_fut => {
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

async fn handle_orbit_escape(
    mode: GlobalMode,
    vel_change: &VelocityChangeTask,
    k: &Arc<KeychainWithOrbit>,
) -> GlobalMode {
    if let GlobalMode::ZonedObjectivePrepMode(obj) = mode.clone() {
        let burn = vel_change.burn();
        FlightComputer::execute_burn(k.f_cont(), vel_change.burn()).await;
        let exp_pos = burn.sequence_pos().last().unwrap();
        let current_pos = k.f_cont().read().await.current_pos();
        let diff = *exp_pos - current_pos;
        let detumble_time_delta = TimeDelta::seconds(burn.detumble_dt() as i64);
        let detumble_dt = PinnedTimeDelay::new(detumble_time_delta - DETUMBLE_TOL);
        println!("[INFO] Orbit Escape done! Expected position {exp_pos}, Actual Position {current_pos}, Diff {diff}");
        // TODO: here we shouldn't use objective.get_imaging_points but something already created,
        // TODO: also the mode change to global mode should happen sometime else
        let (vel, dev) =
            FlightComputer::evaluate_burn(k.f_cont(), burn, obj.get_imaging_points()[0]).await;
        TaskController::calculate_orbit_correction_burn(vel, dev, detumble_dt);
        GlobalMode::ZonedObjectiveRetrievalMode(obj)
    } else {
        println!(
            "[ERROR] Orbit escape change requested, global mode illegal. Skipping velocity change!"
        );
        GlobalMode::MappingMode
    }
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
