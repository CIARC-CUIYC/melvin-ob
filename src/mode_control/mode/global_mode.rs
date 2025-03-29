use crate::flight_control::{
    beacon_controller::BeaconControllerState,
    objective::known_img_objective::KnownImgObjective,
    task::base_task::Task
};
use crate::mode_control::{
    base_mode::BaseMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal, OptOpExitSignal},
};
use crate::{DT_0_STD, fatal, info, log, warn};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use std::mem::discriminant;
use std::{future::Future, pin::Pin, sync::Arc};
use tokio::{sync::{RwLock, watch::Receiver}, task::JoinError};
use tokio_util::sync::CancellationToken;

/// Trait representing a high-level operational mode within the onboard Finite-State-Machine (FSM) architecture.
/// Implementors of [`GlobalMode`] encapsulate full behavioral logic for mode-specific task scheduling,
/// signal handling, and state transitions.
#[async_trait]
pub trait GlobalMode: Sync + Send {
    /// Returns the rationale string for  finishing the current phase due to safe mode entry.
    fn safe_mode_rationale(&self) -> &'static str { "SAFE mode Event!" }
    /// Returns the rationale string for finishing the current phase due to a new Zoned Objective.
    fn new_zo_rationale(&self) -> &'static str { "newly discovered ZO!" }
    /// Returns the rationale string for finishing the current phase due to a new Beacon Objective.
    fn new_bo_rationale(&self) -> &'static str { "newly discovered BO!" }
    /// Returns the rationale string used when the task queue has completed.
    fn tasks_done_rationale(&self) -> &'static str { "tasks list done!" }
    /// Returns the rationale for finishing the task queue and exiting the orbit for ZO Retrieval.
    fn tasks_done_exit_rationale(&self) -> &'static str {
        "tasks list done and exited orbit for ZO Retrieval!"
    }
    /// Returns the rationale for finishing the current phase due to being outside of orbit without a valid reason.
    fn out_of_orbit_rationale(&self) -> &'static str { "out of orbit without purpose!" }
    /// Returns the rationale used for finishing the current phase when a beacon objective has been completed or expired.
    fn bo_done_rationale(&self) -> &'static str { "BO done or expired!" }

    /// Returns the string representation of the current mode.
    fn type_name(&self) -> &'static str;

    /// Initializes the mode with the provided context.
    ///
    /// # Arguments
    /// * `context` - Shared reference to the current mode context.
    ///
    /// # Returns
    /// [`OpExitSignal`] - Signal indicating what action to take after initialization.
    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal;

    /// Executes all tasks in the current task queue in sequence.
    /// Waits for each task’s scheduled time and handles early exit signals such as safe transitions or new objectives.
    ///
    /// # Arguments
    /// * `context` - Shared reference to the current mode context.
    ///
    /// # Returns
    /// * [`OpExitSignal`] - Signal indicating whether to continue or exit the mode.
    #[allow(clippy::cast_sign_loss, clippy::cast_precision_loss)]
    async fn exec_task_queue(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let context_local = Arc::clone(&context);
        let mut tasks = 0;
        while let Some(task) = {
            let sched_arc = context_local.k().t_cont().sched_arc();
            let mut sched_lock = sched_arc.write().await;
            let t = sched_lock.pop_front();
            drop(sched_lock);
            t
        } {
            let due_time = task.t() - Utc::now();
            let task_type = task.task_type();
            info!("TASK {tasks}: {task_type} in  {}s!", due_time.num_seconds());
            while task.t() > Utc::now() + TimeDelta::seconds(2) {
                let context_clone = Arc::clone(&context);
                match self.exec_task_wait(context_clone, task.t()).await {
                    WaitExitSignal::Continue => {}
                    WaitExitSignal::SafeEvent => {
                        return self.safe_handler(context_local).await;
                    }
                    WaitExitSignal::NewZOEvent(obj) => {
                        if let Some(opt) = self.zo_handler(&context, obj).await {
                            return opt;
                        };
                    }
                    WaitExitSignal::BOEvent => {
                        if let Some(opt) = self.bo_event_handler(&context).await {
                            return opt;
                        };
                    }
                };
            }
            let task_delay = (task.t() - Utc::now()).num_milliseconds() as f32 / 1000.0;
            if task_delay.abs() > 2.0 {
                log!("Task {tasks} delayed by {task_delay}s!");
            }
            let context_clone = Arc::clone(&context);
            match self.exec_task(context_clone, task).await {
                ExecExitSignal::Continue => {}
                ExecExitSignal::SafeEvent => {
                    return self.safe_handler(context_local).await;
                }
                ExecExitSignal::NewZOEvent(_) => {
                    fatal!("Unexpected task exit signal!");
                }
            };
            tasks += 1;
        }
        OpExitSignal::Continue
    }

    /// Waits until a task’s due time or handles early exit signals while executing a wait primitive.
    ///
    /// # Arguments
    /// * `context` - Shared reference to the mode context.
    /// * `due` - Scheduled task start time.
    ///
    /// # Returns
    /// * `WaitExitSignal` - Resulting signal from monitoring exit conditions.
    async fn exec_task_wait(&self, context: Arc<ModeContext>, due: DateTime<Utc>)
    -> WaitExitSignal;
    
    /// Executes a single task and returns a signal indicating whether to continue or exit.
    ///
    /// # Arguments
    /// * `context` - Shared reference to the mode context.
    /// * `task` - Task to be executed.
    ///
    /// # Returns
    /// * `ExecExitSignal` - Resulting signal after task execution.
    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal;

    /// Handles unplanned safe mode transition.
    ///
    /// # Arguments
    /// * `context` - Shared reference to the mode context.
    ///
    /// # Returns
    /// * `OpExitSignal` - Signal after executing safe-mode exit logic.
    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal;

    /// Handles the reception of a new Zoned Objective (ZO).
    ///
    /// # Arguments
    /// * `context` - Shared reference to the mode context.
    /// * `obj` - The new `KnownImgObjective` that triggered the transition.
    ///
    /// # Returns
    /// * `OptOpExitSignal` - Optional signal indicating a mode switch or continuation.
    async fn zo_handler(
        &self,
        context: &Arc<ModeContext>,
        obj: KnownImgObjective,
    ) -> OptOpExitSignal;
    
    /// Handles beacon-related events (e.g., mode changes or new signals).
    ///
    /// # Arguments
    /// * `context` - Shared reference to the mode context.
    ///
    /// # Returns
    /// * `OptOpExitSignal` - Optional signal indicating a mode switch or continuation.
    async fn bo_event_handler(&self, context: &Arc<ModeContext>) -> OptOpExitSignal;

    /// Handles cleanup and transition logic when exiting a mode.
    ///
    /// # Arguments
    /// * `context` - Shared reference to the mode context.
    ///
    /// # Returns
    /// * `Box<dyn GlobalMode>` - The next mode to transition into.
    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode>;
}

/// An internal extension trait for [`GlobalMode`] that encapsulates logic specific to
/// time-constrained orbital task execution.
///
/// The [`OrbitalMode`] trait provides utility methods for task waiting, event monitoring,
/// and logging transitions related to orbital activities, such as beacon detection or
/// zoned objective events.
///
/// This trait is not intended to be used directly outside the mode control subsystem.
pub(super) trait OrbitalMode: GlobalMode {
    /// Returns the maximum duration allowed for scheduled task waiting.
    /// If the remaining time until a task is due exceeds this value, the mode may begin
    /// long-wait procedures; otherwise, a fallback short sleep is used.
    ///
    /// # Returns
    /// * [`TimeDelta`] – Maximum duration (default: 10 seconds).
    fn get_max_dt() -> TimeDelta { TimeDelta::seconds(10) }

    /// Returns a reference to the [`BaseMode`] associated with this orbital mode.
    ///
    /// This is used for delegating tasks and behavior such as wait and scheduling primitives.
    ///
    /// # Returns
    /// * `&BaseMode` – The current mode (e.g., Mapping or Beacon Objective Scanning).
    fn base(&self) -> &BaseMode;

    /// Waits until a scheduled task’s due time or returns early if a notable event occurs.
    ///
    /// While waiting, this method concurrently monitors for:
    /// - Safe mode triggers
    /// - New zoned objectives (ZO)
    /// - Beacon state changes (BO)
    ///
    /// It also supports short or long sleep strategies depending on how far the task lies in the future.
    ///
    /// # Arguments
    /// * `context` – Shared reference to the current [`ModeContext`].
    /// * `due` – Scheduled time at which the next task is expected to start.
    ///
    /// # Returns
    /// * [`WaitExitSignal`] – An event indicating why the wait ended (safe event, ZO, or BO, ...).
    async fn exec_task_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
    ) -> WaitExitSignal {
        let safe_mon = context.super_v().safe_mon();
        let mut zo_mon = context.zo_mon().write().await;
        let bo_mon = context.bo_mon();
        let cancel_task = CancellationToken::new();

        let fut: Pin<Box<dyn Future<Output = Result<_, JoinError>> + Send>> =
            if (due - Utc::now()) > Self::get_max_dt() {
                Box::pin(self.base().get_wait(Arc::clone(&context), due, cancel_task.clone()).await)
            } else {
                warn!("Task wait time too short. Just waiting!");
                Box::pin(async {
                    let sleep = (due - Utc::now()).to_std().unwrap_or(DT_0_STD);
                    tokio::time::timeout(sleep, cancel_task.cancelled()).await.ok().unwrap_or(());
                    Ok(())
                })
            };
        let bo_change_signal = self.base().get_rel_bo_event();
        tokio::pin!(fut);
        tokio::select! {
            exit_sig = &mut fut => {
                exit_sig.unwrap_or_else(|_|fatal!("Task wait hung up!"));
                WaitExitSignal::Continue
            },
            () = safe_mon.notified() => {
                cancel_task.cancel();
                fut.await.ok();
                WaitExitSignal::SafeEvent
            },
            msg =  zo_mon.recv() => {
                let img_obj = msg.unwrap_or_else(||fatal!("Objective monitor wait hung up!"));
                cancel_task.cancel();
                fut.await.ok();
                WaitExitSignal::NewZOEvent(img_obj)
            }
            () = Self::monitor_bo_mon_change(bo_change_signal, bo_mon) => {
                cancel_task.cancel();
                fut.await.ok();
                WaitExitSignal::BOEvent
            }

        }
    }

    /// Continuously monitors the [`BeaconController`] state until it changes to the expected value.
    ///
    /// Used to react to asynchronous events in which the beacon scanning mode (e.g., active or inactive)
    /// must trigger a response in the scheduler or mode transition logic.
    ///
    /// # Arguments
    /// * `sig` – The [`BeaconController`] state to wait for.
    /// * `bo_mon` – A watch receiver providing asynchronous access to beacon state changes.
    async fn monitor_bo_mon_change(
        sig: BeaconControllerState,
        bo_mon: &RwLock<Receiver<BeaconControllerState>>,
    ) {
        let mut bo_mon_lock = bo_mon.write().await;
        loop {
            if let Ok(()) = bo_mon_lock.changed().await {
                let sent_sig = bo_mon_lock.borrow_and_update();
                let sent_sig_ref = &*sent_sig;
                if discriminant(&sig) == discriminant(sent_sig_ref) {
                    return;
                }
            }
        }
    }

    /// Logs a beacon-related event and finalizes the orbit at the current satellite position.
    ///
    /// This is used to capture the reason for switching out of the current [`BaseMode`],
    /// typically due to either beacon objective completion or expiry.
    ///
    /// # Arguments
    /// * `context` – Shared reference to the current [`ModeContext`].
    /// * `base` – The [`BaseMode`] that determines the rationale for finishing the orbit segment.
    async fn log_bo_event(&self, context: &Arc<ModeContext>, base: BaseMode) {
        match base {
            BaseMode::BeaconObjectiveScanningMode => context.o_ch_lock().write().await.finish(
                context.k().f_cont().read().await.current_pos(),
                self.new_bo_rationale(),
            ),
            BaseMode::MappingMode => context.o_ch_lock().write().await.finish(
                context.k().f_cont().read().await.current_pos(),
                self.bo_done_rationale(),
            ),
        }
    }
}
