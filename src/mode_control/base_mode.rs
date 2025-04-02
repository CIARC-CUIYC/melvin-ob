use super::mode_context::ModeContext;
use super::signal::{
    PeriodicImagingEndSignal,
    TaskEndSignal::{self, Join, Timestamp},
};
use crate::flight_control::{FlightComputer, FlightState, orbit::IndexedOrbitPosition};
use crate::imaging::CameraAngle;
use crate::objective::BeaconControllerState;
use crate::scheduling::{EndCondition, TaskController, task::SwitchStateTask};
use crate::{DT_0_STD, error, fatal, info, log};
use chrono::{DateTime, TimeDelta, Utc};
use std::{future::Future, pin::Pin, sync::Arc};
use strum_macros::Display;
use tokio::{sync::oneshot, task::JoinHandle, time::Instant};
use tokio_util::sync::CancellationToken;

/// Represents high-level operational modes of the onboard software when in orbit.
/// Each variant encodes different scheduling logic and task handling behavior.
#[derive(Display, Clone, Copy)]
pub(super) enum BaseMode {
    /// Regular mapping mode focused on maximizing imaging coverage.
    MappingMode,
    /// Mode dedicated to get full communications mode coverage while still mapping when possible.
    BeaconObjectiveScanningMode,
}

impl BaseMode {
    /// Default camera angle used during mapping operations.
    const DEF_MAPPING_ANGLE: CameraAngle = CameraAngle::Narrow;

    /// Executes a full mapping acquisition cycle, listening until either a signal or cancellation occurs.
    ///
    /// This function initializes an image acquisition cycle using the default mapping camera angle
    /// and coordinates between the camera controller and various signal channels.
    /// It finalizes by marking orbit coverage and exporting updated coverage data.
    ///
    /// # Arguments
    /// - `context`: A shared reference to a [`ModeContext`] object.
    /// - `end`: A [`TaskEndSignal`]-enum type indicating how the task end condition should be defined.
    /// - `c_tok`: A [`CancellationToken`] that is able to cancel this task with proper cleanup.
    #[allow(clippy::cast_possible_wrap)]
    async fn exec_map(context: Arc<ModeContext>, end: TaskEndSignal, c_tok: CancellationToken) {
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
            FlightComputer::set_angle_wait(Arc::clone(&f_cont_lock), Self::DEF_MAPPING_ANGLE).await;
            let handle = tokio::spawn(async move {
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
                        acq_phase.1.send(sig).unwrap_or_else(|_|fatal!("Receiver hung up!"));
                        join_handle.abort();
                        acq_phase.0.await.ok().unwrap_or(vec![(0, 0)])
                    },
                    _ = &mut join_handle => {
                        let sig = PeriodicImagingEndSignal::KillLastImage;
                        acq_phase.1.send(sig).unwrap_or_else(|_|fatal!("Receiver hung up!"));
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
        let c_orbit_lock = k_loc.c_orbit();
        let mut c_orbit = c_orbit_lock.write().await;
        for (start, end) in &fixed_ranges {
            if start != end {
                c_orbit.mark_done(*start, *end);
            }
        }
        log!(
            "Current discrete Orbit Coverage is {}%.",
            c_orbit.get_coverage() * 100
        );
        c_orbit.try_export_default();
    }

    /// Listens for Beacon Objective communication pings until a timeout or cancellation.
    ///
    /// Uses an event-based listener to process incoming beacon messages.
    /// Automatically terminates based on task completion or shutdown signals.
    ///
    /// # Arguments
    /// - `context`: A shared reference to a `ModeContext` object.
    /// - `end`: A `TaskEndSignal`-enum type indicating how the task end condition should be defined.
    /// - `c_tok`: A `CancellationToken` that is able to cancel this task with proper cleanup.
    async fn exec_comms(context: Arc<ModeContext>, end: TaskEndSignal, c_tok: CancellationToken) {
        let mut event_rx = context.super_v().subscribe_event_hub();

        let mut fut: Pin<Box<dyn Future<Output = ()> + Send>> = match end {
            Timestamp(t) => {
                let due_secs = (t - Utc::now()).to_std().unwrap_or(DT_0_STD);
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
                    context.beac_cont().handle_poss_bo_ping(msg, f_cont).await;
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

    /// Ensures any required preconditions for the current mode are satisfied before scheduling begins.
    ///
    /// # Arguments
    /// - `context`: A shared reference to a `ModeContext` object.
    ///
    /// # Returns
    /// A `DateTime<Utc>` indicating the time when scheduled tasks should start.
    pub(super) async fn handle_sched_preconditions(
        &self,
        context: Arc<ModeContext>,
    ) -> DateTime<Utc> {
        match self {
            BaseMode::MappingMode => FlightComputer::escape_if_comms(context.k().f_cont()).await,
            BaseMode::BeaconObjectiveScanningMode => {
                FlightComputer::get_to_comms(context.k().f_cont()).await
            }
        }
    }

    /// Returns a handle to the scheduling task corresponding to the current operational `BaseMode`-variant.
    ///
    /// - For `MappingMode`, a standard optimal orbit scheduler is used.
    /// - For `BeaconObjectiveScanningMode`, a communications-aware scheduler is launched.
    ///
    /// Depending on the current flight state, this will also launch a mapping or
    /// beacon listening task to run concurrently.
    ///
    /// # Arguments
    /// - `context`: A shared reference to a `ModeContext` object.
    /// - `c_tok`: A `CancellationToken` that is able to cancel this task with proper cleanup.
    /// - `comms_end`: A `DateTime<Utc>` indicating the end of the current comms cycle when in `BeaconObjectiveScanningMode`.
    /// - `end`: An optional `EndCondition` type if i.e. a burn sequence follows to this task list.
    ///
    /// # Returns
    /// A `JoinHandle<()` to join with the scheduling task
    #[must_use]
    pub(super) async fn get_schedule_handle(
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
                    context.beac_cont().last_active_beac_end().await.unwrap_or(Utc::now());
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
            j_handle
        }
    }

    /// Spawns the corresponding primitive for the task wait time.
    ///
    /// The returned handle either:
    /// - `FlightState::Charge`: Waits for the task timeout and exports a full image snapshot.
    /// - `FlightState::Acquisition`: Executes a mapping task.
    /// - `FlightState::Comms`: Executes a beacon listening task.
    ///
    /// This is useful for blocking execution while ensuring mode-specific behavior continues.
    ///
    /// # Arguments
    /// - `context`: A shared reference to a `ModeContext` object.
    /// - `due`: A `DateTime<Utc>` indicating the task wait timeout time.
    /// - `c_tok`: A `CancellationToken` that is able to cancel the spawned task with proper cleanup.
    ///
    /// # Returns
    /// A `JoinHandle<()` to join with the state specific primitive.
    #[must_use]
    pub(super) async fn get_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
        c_tok: CancellationToken,
    ) -> JoinHandle<()> {
        let (current_state, _) = {
            let f_cont = context.k().f_cont();
            let lock = f_cont.read().await;
            (lock.state(), lock.client())
        };
        let c_tok_clone = c_tok.clone();
        let def = Box::pin(async move {
            let sleep = (due - Utc::now()).to_std().unwrap_or(DT_0_STD);
            tokio::time::timeout(sleep, c_tok_clone.cancelled()).await.ok().unwrap_or(());
        });
        let task_fut: Pin<Box<dyn Future<Output = _> + Send>> = match current_state {
            FlightState::Charge => def,
            FlightState::Acquisition => Box::pin(async move {
                Self::exec_map(context, Timestamp(due), c_tok).await;
            }),
            FlightState::Comms => {
                if let Self::BeaconObjectiveScanningMode = self {
                    Box::pin(async move {
                        Self::exec_comms(context, Timestamp(due), c_tok).await;
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

    /// Executes the corresponding primitive for task execution.
    ///
    /// In `GlobalModes` with a corresponding `BaseMode` this handles the logic for `SwitchStateTasks`.
    ///
    /// # Arguments
    /// - `context`: A shared reference to a `ModeContext` object.
    /// - `task`: The corresponding `SwitchStateTask` object.
    pub(super) async fn get_task(&self, context: Arc<ModeContext>, task: SwitchStateTask) {
        let f_cont = context.k().f_cont();
        match task.target_state() {
            FlightState::Acquisition => {
                FlightComputer::set_state_wait(f_cont, FlightState::Acquisition).await;
            }
            FlightState::Charge => {
                let task_handle = async {
                    FlightComputer::set_state_wait(f_cont, FlightState::Charge).await;
                };
                let k_clone = Arc::clone(context.k());
                let export_handle = tokio::spawn(async move {
                    let c_cont = k_clone.c_cont();
                    c_cont
                        .export_full_snapshot()
                        .await
                        .unwrap_or_else(|_| fatal!("Export failed!"));
                    c_cont.create_thumb_snapshot().await.unwrap_or_else(|e| {
                        error!("Error exporting thumb snapshot: {e}.");
                    });
                });
                task_handle.await;
                if export_handle.is_finished() {
                    export_handle.await.unwrap();
                } else {
                    error!("Couldnt finish Map export!");
                    export_handle.abort();
                }
            }
            FlightState::Comms => match self {
                BaseMode::MappingMode => {
                    fatal!("Illegal target state!")
                }
                BaseMode::BeaconObjectiveScanningMode => {
                    FlightComputer::set_state_wait(f_cont, FlightState::Comms).await;
                }
            },
            _ => fatal!("Illegal target state!"),
        }
    }

    /// Returns the relevant `BeaconControllerState` associated with this mode.
    ///
    /// Used to inform beacon-handling logic of the signal that would indicate switching.
    pub(super) fn get_rel_bo_event(self) -> BeaconControllerState {
        match self {
            BaseMode::MappingMode => BeaconControllerState::ActiveBeacons,
            BaseMode::BeaconObjectiveScanningMode => BeaconControllerState::NoActiveBeacons,
        }
    }

    /// Returns the new `BaseMode` following a Beacon Objective Event.
    ///
    /// Used to inform beacon-handling logic of the new state after a signal.
    pub(super) fn bo_event(self) -> Self {
        match self {
            BaseMode::MappingMode => Self::BeaconObjectiveScanningMode,
            BaseMode::BeaconObjectiveScanningMode => Self::MappingMode,
        }
    }
}
