use crate::flight_control::{
    flight_computer::FlightComputer,
    objective::{
        beacon_objective::BeaconObjective, known_img_objective::KnownImgObjective,
        objective_base::ObjectiveBase, objective_type::ObjectiveType,
    },
    task::base_task::{BaseTask, Task},
};
use crate::mode_control::mode::zo_prep_mode::ZOPrepMode;
use crate::mode_control::{
    base_mode::{BaseMode, BaseWaitExitSignal},
    mode::global_mode::{GlobalMode, OrbitalMode},
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use crate::{fatal, obj};
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

    async fn exec_task_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
    ) -> WaitExitSignal {
        <Self as OrbitalMode>::exec_task_wait(self, context, due).await
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

    async fn objective_handler(
        &self,
        context: Arc<ModeContext>,
        obj: ObjectiveBase,
    ) -> Option<OpExitSignal> {
        match obj.obj_type() {
            ObjectiveType::Beacon { .. } => {
                let b_obj = BeaconObjective::new(
                    obj.id(),
                    String::from(obj.name()),
                    obj.start(),
                    obj.end(),
                );
                obj!("Found new Beacon Objective {}!", obj.id());
                if let Some(base) = self.base.handle_b_o(&context, b_obj).await {
                    context.o_ch_lock().write().await.finish(
                        context.k().f_cont().read().await.current_pos(),
                        self.new_bo_rationale(),
                    );
                    Some(OpExitSignal::ReInit(Box::new(Self { base })))
                } else {
                    None
                }
            }
            ObjectiveType::KnownImage {
                zone,
                optic_required,
                coverage_required,
            } => {
                obj!("Found new Zoned Objective {}!", obj.id());
                let k_obj = KnownImgObjective::new(
                    obj.id(),
                    String::from(obj.name()),
                    obj.start(),
                    obj.end(),
                    *zone,
                    *optic_required,
                    *coverage_required,
                );
                context.o_ch_lock().write().await.finish(
                    context.k().f_cont().read().await.current_pos(),
                    self.new_zo_rationale(),
                );
                ZOPrepMode::from_obj(context, k_obj, self.base.clone())
                    .await
                    .map(|mode| OpExitSignal::ReInit(Box::new(mode)))
            }
        }
    }

    async fn b_o_done_handler(
        &self,
        context: Arc<ModeContext>,
        b_sig: BaseWaitExitSignal,
    ) -> OpExitSignal {
        match b_sig {
            BaseWaitExitSignal::Continue => OpExitSignal::Continue,
            BaseWaitExitSignal::ReturnAllDone => {
                context.o_ch_lock().write().await.finish(
                    context.k().f_cont().read().await.current_pos(),
                    self.bo_done_rationale(),
                );
                OpExitSignal::ReInit(Box::new(Self {
                    base: BaseMode::MappingMode,
                }))
            }
            BaseWaitExitSignal::ReturnSomeLeft => {
                context.o_ch_lock().write().await.finish(
                    context.k().f_cont().read().await.current_pos(),
                    self.bo_left_rationale(),
                );
                OpExitSignal::ReInit(Box::new(self.clone()))
            }
        }
    }

    async fn exit_mode(&self, c: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        let handler = c.k().f_cont().read().await.client();
        Box::new(Self {
            base: self.base.exit_base(handler).await,
        })
    }
}
