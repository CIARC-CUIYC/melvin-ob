use crate::flight_control::{
    flight_computer::FlightComputer,
    objective::known_img_objective::KnownImgObjective,
    task::base_task::{BaseTask, Task},
};
use crate::mode_control::mode::zo_prep_mode::ZOPrepMode;
use crate::mode_control::signal::OptOpExitSignal;
use crate::mode_control::{
    base_mode::BaseMode,
    mode::global_mode::{GlobalMode, OrbitalMode},
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use crate::{fatal, obj, warn};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use std::sync::Arc;
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub struct InOrbitMode {
    base: BaseMode,
}

impl InOrbitMode {
    const MODE_NAME: &'static str = "InOrbitMode";
    const MAX_WAIT_TIMEDELTA: TimeDelta = TimeDelta::seconds(10);

    pub fn new(base: BaseMode) -> Self { Self { base } }
}

impl OrbitalMode for InOrbitMode {
    fn base(&self) -> &BaseMode { &self.base }
}

#[async_trait]
impl GlobalMode for InOrbitMode {
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

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

    async fn exec_task_wait(&self, c: Arc<ModeContext>, due: DateTime<Utc>) -> WaitExitSignal {
        <Self as OrbitalMode>::exec_task_wait(self, c, due).await
    }

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

    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        FlightComputer::escape_safe(context.k().f_cont(), false).await;
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.safe_mode_rationale(),
        );
        OpExitSignal::ReInit(Box::new(self.clone()))
    }

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

    async fn bo_event_handler(&self, context: &Arc<ModeContext>) -> OptOpExitSignal {
        let base = self.base.bo_event();
        self.log_bo_event(context, base).await;
        Some(OpExitSignal::ReInit(Box::new(Self { base })))
    }

    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.tasks_done_rationale(),
        );
        Box::new(self.clone())
    }
}
