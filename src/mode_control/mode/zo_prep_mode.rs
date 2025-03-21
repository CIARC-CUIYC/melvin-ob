use crate::flight_control::{
    flight_computer::FlightComputer,
    objective::{
        beacon_objective::BeaconObjective, known_img_objective::KnownImgObjective,
        objective_base::ObjectiveBase, objective_type::ObjectiveType,
    },
    orbit::BurnSequence,
    task::{
        base_task::{BaseTask, Task},
        TaskController,
        end_condition::EndCondition,
        vel_change_task::VelocityChangeTaskRationale::OrbitEscape
    },
};
use crate::mode_control::{
    base_mode::{BaseMode, BaseWaitExitSignal},
    mode::{
        global_mode::{GlobalMode, OrbitalMode},
        in_orbit_mode::InOrbitMode,
    },
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use crate::{error, fatal, info, log, obj};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use std::sync::Arc;
use fixed::types::I32F32;
use tokio_util::sync::CancellationToken;
use crate::flight_control::common::vec2d::Vec2D;
use crate::mode_control::mode::zo_retrieval_mode::ZORetrievalMode;

#[derive(Clone)]
pub struct ZOPrepMode {
    base: BaseMode,
    exit_burn: BurnSequence,
    target: KnownImgObjective,
    targets: Vec<Vec2D<I32F32>>,
    left_orbit: bool,
}

impl ZOPrepMode {
    const MODE_NAME: &'static str = "ZOPrepMode";
    const MIN_REPLANNING_DT: TimeDelta = TimeDelta::seconds(500);

    #[allow(clippy::cast_possible_wrap)]
    pub async fn from_obj(
        context: Arc<ModeContext>,
        zo: KnownImgObjective,
        mut base: BaseMode,
    ) -> Option<Self> {
        let targets = zo.get_imaging_points();
        let exit_burn = if zo.min_images() == 1 {
            let due = zo.end();
            let current_vel = context.k().f_cont().read().await.current_vel();
            TaskController::calculate_single_target_burn_sequence(
                context.o_ch_clone().await.i_entry(),
                current_vel,
                targets[0],
                due,
            )
            .await
        } else {
            // TODO: multiple imaging points?
            fatal!("Zoned Objective with multiple images not yet supported");
        }?;
        let entry_pos = exit_burn.sequence_pos().first().unwrap();
        let exit_pos = exit_burn.sequence_pos().last().unwrap();
        let entry_t = exit_burn.start_i().t().format("%H:%M:%S").to_string();
        let exit_vel = exit_burn.sequence_vel().last().unwrap();
        let tar = zo.get_imaging_points()[0];
        info!("Calculated Burn Sequence for Zoned Objective: {}", zo.id());
        log!("Entry at {entry_t}, Position will be {entry_pos}");
        log!("Exit after {}s, Position will be {exit_pos}", exit_burn.acc_dt());
        log!("Exit Velocity will be {exit_vel} aiming for target at {tar}. Detumble time is {}s.",  exit_burn.detumble_dt());
        log!("Whole BS: {:?}", exit_burn);
        if let BaseMode::BeaconObjectiveScanningMode(_) = base {
            let burn_start = exit_burn.start_i().t();
            let worst_case_first_comms_end = {
                let to_comms = 
                    FlightComputer::get_to_comms_dt_est(context.k().f_cont()).await;
                Utc::now() + to_comms.1 + TimeDelta::seconds(TaskController::IN_COMMS_SCHED_SECS as i64)
            };
            if worst_case_first_comms_end + TimeDelta::seconds(10) > burn_start {
                base = BaseMode::MappingMode;
            }
        }
        Some(ZOPrepMode { base, exit_burn, target: zo, left_orbit: false , targets})
    }
    
    fn new_base(&self, base: BaseMode) -> Self {
        Self {
            base,
            targets: self.targets.clone(),
            exit_burn: self.exit_burn.clone(),
            target: self.target.clone(),
            left_orbit: self.left_orbit,
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
        let comms_end = self.base.handle_sched_preconditions(Arc::clone(&context)).await;
        let end = EndCondition::from_burn(&self.exit_burn);
        let sched_handle = {
            let cancel_clone = cancel_task.clone();
            self.base.get_schedule_handle(Arc::clone(&context), cancel_clone, comms_end, Some(end)).await
        };
        tokio::pin!(sched_handle);
        let safe_mon = context.super_v().safe_mon();
        tokio::select!(
            _ = &mut sched_handle => {
                info!("Additionally scheduling Orbit Escape Burn Sequence!");
                context.k().t_cont().schedule_vel_change(self.exit_burn.clone(), OrbitEscape).await;
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
            BaseTask::ChangeVelocity(vel_change) => {
                let pos = context.k().f_cont().read().await.current_pos();
                log!(
                    "Burn started at Pos {pos}. Expected Position was: {}.",
                    vel_change.burn().sequence_pos()[0]
                );
                FlightComputer::execute_burn(context.k().f_cont(), vel_change.burn()).await;
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
        let new = Self::from_obj(context, self.target.clone(), self.base.clone()).await;
        OpExitSignal::ReInit(
            new.map_or(Box::new(InOrbitMode::new(self.base.clone())), |b| {
                Box::new(b)
            }),
        )
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
                    Some(OpExitSignal::ReInit(Box::new(self.new_base(base))))
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
                let burn_dt_cond = self.exit_burn.start_i().t() - Utc::now() > Self::MIN_REPLANNING_DT;
                if k_obj.end() < self.target.end() && burn_dt_cond {
                    let new_obj_mode = Self::from_obj(context, k_obj, self.base.clone()).await;
                    if let Some(prep_mode) = new_obj_mode {
                        // TODO: store current zoned objective for later retrieval
                        return Some(OpExitSignal::ReInit(Box::new(prep_mode)));
                    }
                }
                None
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
                OpExitSignal::ReInit(Box::new(self.new_base(BaseMode::MappingMode)))
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

    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        let handler = context.k().f_cont().read().await.client();
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.tasks_done_exit_rationale(),
        );
        if self.left_orbit {
            Box::new(ZORetrievalMode::new(self.target.clone(), self.targets.clone()))
        } else {
            error!("ZOPrepMode::exit_mode called without left_orbit flag set!");
            Box::new(InOrbitMode::new(self.base.exit_base(handler).await))
        }
        
    }
}
