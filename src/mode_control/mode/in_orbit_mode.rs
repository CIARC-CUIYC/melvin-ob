use crate::scheduling::task::{BaseTask, Task};
use crate::objective::KnownImgObjective;
use crate::flight_control::FlightComputer;
use super::{zo_prep_mode::ZOPrepMode, global_mode::{GlobalMode, OrbitalMode}};
use crate::mode_control::{
    base_mode::BaseMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal, OptOpExitSignal},
};
use crate::{fatal, obj, warn};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use std::sync::Arc;
use tokio_util::sync::CancellationToken;

/// [`InOrbitMode`] is an implementation of [`GlobalMode`] and [`OrbitalMode`] that governs normal
/// in-orbit operations such as transitioning between flight states, listening for event-driven
/// interrupts (e.g., Safe Mode, ZO, BO), and executing scheduled task queues specific to the [`InOrbitMode`].
///
/// This mode represents the satellite operating in a stable orbital state, performing mapping or communication tasks.
///
/// The mode is interruptible and reactive to new objectives, flight events, or beacon detections.
#[derive(Clone)]
pub struct InOrbitMode {
    /// The base operational context (e.g. Mapping or Beacon Objective Scanning).
    base: BaseMode,
}

impl InOrbitMode {
    /// The internal name of the mode used for logging and identification.
    const MODE_NAME: &'static str = "InOrbitMode";

    /// Constructs a new [`InOrbitMode`] instance using the given [`BaseMode`].
    ///
    /// # Arguments
    /// * `base` – The high-level operational context (e.g., `MappingMode`).
    ///
    /// # Returns
    /// * [`InOrbitMode`] – The initialized mode.
    pub fn new(base: BaseMode) -> Self { Self { base } }
}

impl OrbitalMode for InOrbitMode {
    /// Returns a reference to the current [`BaseMode`] context.
    fn base(&self) -> &BaseMode { &self.base }
}

#[async_trait]
impl GlobalMode for InOrbitMode {
    /// Returns the static name of this mode.
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    /// Initializes the mode by running scheduling logic and listening for early exit signals.
    ///
    /// Reacts to Safe Mode signals and reinitializes if needed. Otherwise, continues
    /// with task list execution.
    ///
    /// # Arguments
    /// * `context` – The shared execution context for the mode.
    ///
    /// # Returns
    /// * [`OpExitSignal`] – Signal indicating if the mode should continue or reinitialize.
    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let cancel_task = CancellationToken::new();
        let comms_end = self.base.handle_sched_preconditions(Arc::clone(&context)).await;
        let sched_handle = {
            let cancel_clone = cancel_task.clone();
            self.base.get_schedule_handle(Arc::clone(&context), cancel_clone, comms_end, None).await
        };
        tokio::pin!(sched_handle);
        let safe_mon = context.super_v().safe_mon();
        tokio::select!(
            _ = &mut sched_handle => {
                context.k().con().send_tasklist().await;
            },
            () = safe_mon.notified() => {
                cancel_task.cancel();
                sched_handle.await.ok();

                // Return to mapping mode
                return OpExitSignal::ReInit(Box::new(self.clone()))
            }
        );
        OpExitSignal::Continue
    }

    /// Delegates to the default orbital task wait logic, including early exits.
    ///
    /// # Arguments
    /// * `c` – The mode context.
    /// * `due` – Scheduled time of task execution.
    ///
    /// # Returns
    /// * `WaitExitSignal` – Signal indicating why the wait was interrupted (if at all).
    async fn exec_task_wait(&self, c: Arc<ModeContext>, due: DateTime<Utc>) -> WaitExitSignal {
        <Self as OrbitalMode>::exec_task_wait(self, c, due).await
    }

    /// Executes a single scheduled task.
    ///
    /// Only state-switching tasks are valid in this mode. Other task types will cause a fatal error.
    ///
    /// # Arguments
    /// * `context` – Mode context for task execution.
    /// * `task` – Task to execute.
    ///
    /// # Returns
    /// * `ExecExitSignal::Continue` – Always returned unless fatal occurs.
    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal {
        match task.task_type() {
            BaseTask::SwitchState(switch) => self.base.get_task(context, *switch).await,
            _ => {
                fatal!(
                    "Illegal task type {} for state {}!",
                    task.task_type(),
                    Self::MODE_NAME
                )
            }
        }
        ExecExitSignal::Continue
    }

    /// Handles the transition into [`FlightState::Safe`], executing a fallback escape sequence 
    /// and finishing the orbit phase.
    ///
    /// # Arguments
    /// * `context` – Mode context for safe handling.
    ///
    /// # Returns
    /// * `OpExitSignal::ReInit` – Always reinitializes the current mode.
    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        FlightComputer::escape_safe(context.k().f_cont(), false).await;
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.safe_mode_rationale(),
        );
        OpExitSignal::ReInit(Box::new(self.clone()))
    }

    /// Handles the detection of a new Zoned Objective.
    ///
    /// Attempts to switch to a `ZOPrepMode`. If the objective is unreachable, logs a warning and continues.
    ///
    /// # Arguments
    /// * `c` – Shared context.
    /// * `obj` – The newly received zoned objective.
    ///
    /// # Returns
    /// * `Some(OpExitSignal::ReInit)` – If transition to `ZOPrepMode` is feasible.
    /// * `None` – If the objective is not reachable (e.g., burn not possible).
    async fn zo_handler(&self, c: &Arc<ModeContext>, obj: KnownImgObjective) -> OptOpExitSignal {
        let id = obj.id();
        obj!("Found new Zoned Objective {id}!");

        if let Some(zo_mode) = ZOPrepMode::from_obj(c, obj, self.base).await {
            c.o_ch_lock().write().await.finish(
                c.k().f_cont().read().await.current_pos(),
                self.new_zo_rationale(),
            );
            Some(OpExitSignal::ReInit(Box::new(zo_mode)))
        } else {
            warn!("Skipping Objective, burn not feasible.");
            None
        }
    }

    /// Handles a beacon objective event by toggling to the complementary base mode.
    /// Logs the beacon transition reason and returns a reinit signal to the new mode.
    ///
    /// # Arguments
    /// * `context` – Shared mode context.
    ///
    /// # Returns
    /// * `Some(OpExitSignal::ReInit)` – Always switches to the next beacon mode.
    async fn bo_event_handler(&self, context: &Arc<ModeContext>) -> OptOpExitSignal {
        let base = self.base.bo_event();
        self.log_bo_event(context, base).await;
        Some(OpExitSignal::ReInit(Box::new(Self { base })))
    }

    /// Performs final cleanup when exiting the mode and marks the phase as finished.
    ///
    /// # Arguments
    /// * `context` – Shared context.
    ///
    /// # Returns
    /// * `Box<dyn GlobalMode>` – A boxed copy of the current mode.
    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.tasks_done_rationale(),
        );
        Box::new(self.clone())
    }
}
