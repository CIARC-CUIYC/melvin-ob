use crate::flight_control::{
    camera_state::CameraAngle,
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::IndexedOrbitPosition,
    task::{TaskController, switch_state_task::SwitchStateTask},
};
use crate::{error, fatal, info, log};

use crate::flight_control::task::end_condition::EndCondition;
use crate::mode_control::base_mode::TaskEndSignal::{Join, Timestamp};
use crate::mode_control::mode_context::ModeContext;
use chrono::{DateTime, TimeDelta, Utc};
use std::{future::Future, pin::Pin, sync::Arc, time::Duration};
use strum_macros::Display;
use tokio::{sync::oneshot, task::JoinHandle, time::Instant};
use tokio_util::sync::CancellationToken;
use crate::flight_control::beacon_controller::BeaconControllerState;
use crate::mode_control::signal::BaseWaitExitSignal;

pub(crate) enum TaskEndSignal {
    Timestamp(DateTime<Utc>),
    Join(JoinHandle<()>),
}

#[derive(Debug)]
pub enum PeriodicImagingEndSignal {
    KillNow,
    KillLastImage,
}

#[derive(Display, Clone)]
pub enum BaseMode {
    MappingMode,
    BeaconObjectiveScanningMode,
}

impl BaseMode {
    const DT_0_STD: Duration = Duration::from_secs(0);
    const MAX_MSG_DT: TimeDelta = TimeDelta::milliseconds(300);
    const DEF_MAPPING_ANGLE: CameraAngle = CameraAngle::Narrow;
    const BO_MSG_COMM_PROLONG_STD: Duration = Duration::from_secs(60);
    const BO_MSG_COMM_PROLONG: TimeDelta = TimeDelta::seconds(60);
    const MIN_COMM_DT: Duration = Duration::from_secs(60);
    const MAX_COMM_INTERVAL: Duration = Duration::from_secs(500);
    const MAX_COMM_PROLONG_RESCHEDULE: TimeDelta = TimeDelta::seconds(2);
    const BEACON_OBJ_RETURN_MIN_DELAY: TimeDelta = TimeDelta::minutes(10);

    #[allow(clippy::cast_possible_wrap)]
    pub async fn exec_map(context: Arc<ModeContext>, end: TaskEndSignal, c_tok: CancellationToken) {
        let end_t = {
            match end {
                Timestamp(dt) => dt,
                Join(_) => Utc::now() + TimeDelta::seconds(10000),
            }
        };
        let o_ch_clone = context.o_ch_clone().await;
        let acq_phase = {
            let f_cont_lock = Arc::clone(&context.k().f_cont());
            let (tx, rx) = oneshot::channel();
            let i_start = o_ch_clone.i_entry().new_from_pos(f_cont_lock.read().await.current_pos());
            let k_clone = Arc::clone(context.k());
            let img_dt = o_ch_clone.img_dt();
            let handle = tokio::spawn(async move {
                FlightComputer::set_angle_wait(Arc::clone(&f_cont_lock), Self::DEF_MAPPING_ANGLE)
                    .await;
                k_clone
                    .c_cont()
                    .execute_acquisition_cycle(
                        f_cont_lock,
                        k_clone.con(),
                        (end_t, rx),
                        img_dt,
                        i_start.index(),
                    )
                    .await
            });
            (handle, tx)
        };

        let ranges = {
            if let Join(join_handle) = end {
                tokio::pin!(join_handle);
                tokio::select! {
                    () = c_tok.cancelled() => {
                        let sig = PeriodicImagingEndSignal::KillNow;
                        acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        join_handle.abort();
                        acq_phase.0.await.ok().unwrap_or(vec![(0, 0)])
                    },
                    _ = &mut join_handle => {
                        let sig = PeriodicImagingEndSignal::KillLastImage;
                        acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        acq_phase.0.await.ok().unwrap_or(vec![(0, 0)])
                    }
                }
            } else {
                let img_fut = acq_phase.0;
                tokio::pin!(img_fut);
                tokio::select! {
                    () = c_tok.cancelled() => {
                        let sig = PeriodicImagingEndSignal::KillNow;
                        acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        img_fut.await.ok().unwrap_or(vec![(0, 0)])
                    }
                    res = &mut img_fut => {
                        res.ok().unwrap_or(vec![(0, 0)])
                    }
                }
            }
        };
        let fixed_ranges =
            IndexedOrbitPosition::map_ranges(&ranges, o_ch_clone.i_entry().period() as isize);
        let and = if let Some(r) = ranges.get(1) {
            format!(" and {} - {}", r.0, r.1)
        } else {
            String::new()
        };
        log!("Marking done: {} - {}{and}", ranges[0].0, ranges[0].1);
        let k_loc = Arc::clone(context.k());
        tokio::spawn(async move {
            let c_orbit_lock = k_loc.c_orbit();
            let mut c_orbit = c_orbit_lock.write().await;
            for (start, end) in &fixed_ranges {
                if start != end {
                    c_orbit.mark_done(*start, *end);
                }
            }
        });
    }

    async fn exec_comms(context: Arc<ModeContext>, due: TaskEndSignal, c_tok: CancellationToken) {
        let mut event_rx = context.super_v().subscribe_event_hub();
        let due_t = { if let Timestamp(t) = due { Some(t) } else { None } };

        let mut fut: Pin<Box<dyn Future<Output = ()> + Send>> = match due {
            Timestamp(t) => {
                let due_secs = (t - Utc::now()).to_std().unwrap_or(Self::DT_0_STD);
                Box::pin(tokio::time::sleep_until(Instant::now() + due_secs))
            }
            Join(join_handle) => Box::pin(async { join_handle.await.ok().unwrap() }),
        };

        let start = Utc::now();
        info!("Starting Comms Listener.");
        loop {
            tokio::select! {
                // Wait for a message
                Ok(msg) = event_rx.recv() => {
                    let f_cont = context.k().f_cont();
                    let prolong = context.beac_cont().handle_poss_bo_ping(msg, due_t, f_cont).await;

                    if prolong {
                    fut = Box::pin(tokio::time::sleep_until(
                        Instant::now() + Self::BO_MSG_COMM_PROLONG_STD,
                    ));
                        }
                }
                // If the timeout expires, exit
                () = &mut fut => {
                    log!("Comms Deadline reached after {}s. Stopping listener.",
                    (Utc::now() - start).num_seconds());
                    break;
                },
                // If the task gets cancelled exit with the updated beacon vector
                () = c_tok.cancelled() => {
                    log!("Comms Listener cancelled. Stopping listener.");
                    break;
                }
            }
        }
    }

    pub async fn handle_sched_preconditions(&self, context: Arc<ModeContext>) -> DateTime<Utc> {
        match self {
            BaseMode::MappingMode => FlightComputer::escape_if_comms(context.k().f_cont()).await,
            BaseMode::BeaconObjectiveScanningMode => {
                FlightComputer::get_to_comms(context.k().f_cont()).await
            }
        }
    }

    pub async fn get_schedule_handle(
        &self,
        context: Arc<ModeContext>,
        c_tok: CancellationToken,
        comms_end: DateTime<Utc>,
        end: Option<EndCondition>,
    ) -> JoinHandle<()> {
        let k = Arc::clone(context.k());
        let o_ch = context.o_ch_clone().await;
        let j_handle = match self {
            BaseMode::MappingMode => tokio::spawn(TaskController::sched_opt_orbit(
                k.t_cont(),
                k.c_orbit(),
                k.f_cont(),
                o_ch.i_entry(),
                end,
            )),
            BaseMode::BeaconObjectiveScanningMode => {
                let last_obj_end =
                    context.beac_cont().last_active_beac_end().await.unwrap_or(Utc::now())
                        + Self::BEACON_OBJ_RETURN_MIN_DELAY;
                tokio::spawn(TaskController::sched_opt_orbit_w_comms(
                    k.t_cont(),
                    k.c_orbit(),
                    k.f_cont(),
                    o_ch.i_entry(),
                    last_obj_end,
                    comms_end,
                    end,
                ))
            }
        };
        let state = context.k().f_cont().read().await.state();
        if state == FlightState::Acquisition {
            tokio::spawn(BaseMode::exec_map(context, Join(j_handle), c_tok))
        } else if state == FlightState::Comms {
            match self {
                BaseMode::MappingMode => fatal!("Illegal state ({state})!"),
                BaseMode::BeaconObjectiveScanningMode => {
                    tokio::spawn(BaseMode::exec_comms(context, Join(j_handle), c_tok))
                }
            }
        } else {
            tokio::spawn(async { j_handle.await.ok().unwrap() })
        }
    }

    pub async fn get_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
        c_tok: CancellationToken,
    ) -> JoinHandle<BaseWaitExitSignal> {
        let (current_state, _) = {
            let f_cont = context.k().f_cont();
            let lock = f_cont.read().await;
            (lock.state(), lock.client())
        };
        let c_tok_clone = c_tok.clone();
        let def = Box::pin(async move {
            let sleep = (due - Utc::now()).to_std().unwrap_or(Self::DT_0_STD);
            tokio::time::timeout(sleep, c_tok_clone.cancelled()).await.ok().unwrap_or(());
            BaseWaitExitSignal::Continue
        });
        let task_fut: Pin<Box<dyn Future<Output = BaseWaitExitSignal> + Send>> = match current_state
        {
            FlightState::Charge => def,
            FlightState::Acquisition => Box::pin(async move {
                Self::exec_map(context, Timestamp(due), c_tok).await;
                BaseWaitExitSignal::Continue
            }),
            FlightState::Comms => {
                if let Self::BeaconObjectiveScanningMode = self {
                    Box::pin(async move {
                        Self::exec_comms(context, Timestamp(due), c_tok).await;
                        if Utc::now() > due + Self::MAX_COMM_PROLONG_RESCHEDULE || Utc::now() < due - Self::MAX_COMM_PROLONG_RESCHEDULE {
                            log!("Prolonging or shortening detected. Rescheduling.");
                            BaseWaitExitSignal::ReSchedule
                        } else {
                            BaseWaitExitSignal::Continue
                        }
                    })
                } else {
                    error!("Not in Beacon Objective Scanning Mode. Waiting for Comms to end.");
                    def
                }
            }
            _ => {
                fatal!("Illegal state ({current_state})!")
            }
        };
        tokio::spawn(task_fut)
    }

    pub async fn get_task(&self, context: Arc<ModeContext>, task: SwitchStateTask) {
        match task.target_state() {
            FlightState::Acquisition => {
                FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Acquisition)
                    .await;
            }
            FlightState::Charge => {
                let join_handle = async {
                    FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Charge).await;
                };
                let k_clone = Arc::clone(context.k());
                tokio::spawn(async move {
                    k_clone.c_cont().create_full_snapshot().await.expect("Export failed!");
                });
                join_handle.await;
            }
            FlightState::Comms => match self {
                BaseMode::MappingMode => {
                    fatal!("Illegal target state!")
                }
                BaseMode::BeaconObjectiveScanningMode => {
                    FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Comms).await;
                }
            },
            _ => fatal!("Illegal target state!"),
        }
    }
    
    pub fn get_rel_bo_event(&self) -> BeaconControllerState {
        match self {
            BaseMode::MappingMode => BeaconControllerState::ActiveBeacons,
            BaseMode::BeaconObjectiveScanningMode => BeaconControllerState::NoActiveBeacons,
        }
    }
    
    pub fn bo_event(&self) -> Self {
        match self {
            BaseMode::MappingMode => Self::BeaconObjectiveScanningMode,
            BaseMode::BeaconObjectiveScanningMode => Self::MappingMode,
        }
    }
}
