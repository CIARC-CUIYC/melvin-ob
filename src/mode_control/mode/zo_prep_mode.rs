use crate::flight_control::{
    flight_computer::FlightComputer,
    objective::{
        beacon_objective::BeaconObjective, known_img_objective::KnownImgObjective,
        objective_base::ObjectiveBase, objective_type::ObjectiveType,
    },
    task::{TaskController, base_task::{BaseTask, Task}},
    orbit::BurnSequence,
};
use crate::mode_control::{
    base_mode::{BaseMode, BaseWaitExitSignal},
    mode::{in_orbit_mode::InOrbitMode, global_mode::{GlobalMode, OrbitalMode}},
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
    mode_context::ModeContext,
};
use crate::{error, fatal, log, obj};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use std::{pin::Pin, sync::Arc};
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub struct ZOPrepMode {
    base: BaseMode,
    exit_burn: BurnSequence,
    target: KnownImgObjective,
}

impl ZOPrepMode {
    const MODE_NAME: &'static str = "ZOPrepMode";

    pub async fn from_obj(context: Arc<ModeContext>, zo: KnownImgObjective) -> Self {
        // TODO: dont unwrap exit burn but handle error if no burn is possible
        let exit_burn = if zo.min_images() == 1 {
            let target_pos = zo.get_imaging_points()[0];
            let due = zo.end();
            TaskController::calculate_single_target_burn_sequence(
                context.o_ch_clone().await.i_entry(),
                Arc::clone(&context.k().f_cont()),
                target_pos,
                due,
            )
            .await
            .unwrap()
        } else {
            // TODO: multiple imaging points?
            fatal!("Zoned Objective with multiple images not yet supported");
        };

        Self {
            base: BaseMode::MappingMode,
            exit_burn,
            target: zo,
        }
    }
}

impl OrbitalMode for ZOPrepMode {
    fn base(&self) -> &BaseMode { &self.base }
}

#[async_trait]
impl GlobalMode for ZOPrepMode {
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let cancel_task = CancellationToken::new();
        self.base.handle_sched_preconditions(Arc::clone(&context)).await;
        let sched_handle = {
            let cancel_clone = cancel_task.clone();
            self.base.get_schedule_handle(Arc::clone(&context), cancel_clone).await
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
    
    async fn exec_task_wait(&self, context: Arc<ModeContext>, due: DateTime<Utc>) -> WaitExitSignal {
        <Self as OrbitalMode>::exec_task_wait(self, context, due).await
    }

    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal {
        match task.task_type() {
            BaseTask::SwitchState(switch) => self.base.get_task(context, *switch).await,
            BaseTask::ChangeVelocity(vel_change) => {
                let pos = context.k().f_cont().read().await.current_pos();
                log!(
                    "Burn started at Pos {pos}. Expected Position was: {}.",
                    vel_change.burn().sequence_pos()[0]
                );
                FlightComputer::execute_burn(context.k().f_cont(), vel_change.burn()).await;
                return ExecExitSignal::ExitedOrbit(self.target.clone());
            }
            BaseTask::TakeImage(_) => fatal!(
                "Illegal task type {} for state {}!",
                task.task_type(),
                Self::MODE_NAME
            ),
        }
        ExecExitSignal::Continue
    }

    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        FlightComputer::escape_safe(context.k().f_cont()).await;
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.safe_mode_rationale(),
        );
        OpExitSignal::ReInit(Box::new(Self::from_obj(context, self.target.clone()).await))
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
                // TODO: move this to external beacon handler
                obj!("Found new Beacon Objective {}!", obj.id());
                if let Some(base) = self.base.handle_b_o(&context, b_obj).await {
                    context.o_ch_lock().write().await.finish(
                        context.k().f_cont().read().await.current_pos(),
                        self.new_bo_rationale(),
                    );
                    Some(OpExitSignal::ReInit(Box::new(self.clone())))
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
                // TODO: return ZonedObjectivePrepMode
                None
                //OpExitSignal::ReInit(Box::New())
            }
        }
    }

    async fn b_o_done_handler(
        &self,
        context: Arc<ModeContext>,
        b_sig: BaseWaitExitSignal,
    ) -> OpExitSignal {
        // TODO: move to BeaconHandler
        match b_sig {
            BaseWaitExitSignal::Continue => OpExitSignal::Continue,
            BaseWaitExitSignal::ReturnAllDone => {
                context.o_ch_lock().write().await.finish(
                    context.k().f_cont().read().await.current_pos(),
                    self.bo_done_rationale(),
                );
                OpExitSignal::ReInit(Box::new(Self {
                    base: BaseMode::MappingMode,
                    exit_burn: self.exit_burn.clone(),
                    target: self.target.clone(),
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
        error!("This mode should never be ended manually!");
        let handler = c.k().f_cont().read().await.client();
        Box::new(InOrbitMode::new(self.base.exit_base(handler).await))
    }
}
