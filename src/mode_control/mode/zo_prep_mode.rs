use crate::flight_control::orbit::ExitBurnResult;
use crate::flight_control::{
    flight_computer::FlightComputer,
    objective::known_img_objective::KnownImgObjective,
    task::{
        TaskController,
        base_task::{BaseTask, Task},
        end_condition::EndCondition,
        vel_change_task::VelocityChangeTaskRationale::OrbitEscape,
    },
};
use crate::mode_control::mode::zo_retrieval_mode::ZORetrievalMode;
use crate::mode_control::{
    base_mode::BaseMode,
    mode::{
        global_mode::{GlobalMode, OrbitalMode},
        in_orbit_mode::InOrbitMode,
    },
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use crate::{error, fatal, info, log};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use tokio_util::sync::CancellationToken;

pub struct ZOPrepMode {
    base: BaseMode,
    exit_burn: ExitBurnResult,
    target: KnownImgObjective,
    left_orbit: AtomicBool,
}

impl Clone for ZOPrepMode {
    fn clone(&self) -> Self {
        Self {
            base: self.base.clone(),
            exit_burn: self.exit_burn.clone(),
            target: self.target.clone(),
            left_orbit: AtomicBool::new(self.left_orbit.load(Ordering::Acquire)),
        }
    }
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
            let (current_vel, fuel_left) = {
                let f_cont_lock = context.k().f_cont();
                let f_cont = f_cont_lock.read().await;
                (f_cont.current_vel(), f_cont.fuel_left())
            };
            TaskController::calculate_single_target_burn_sequence(
                context.o_ch_clone().await.i_entry(),
                current_vel,
                targets[0],
                due,
                fuel_left,
            )
            .await
        } else {
            // TODO: multiple imaging points?
            fatal!("Zoned Objective with multiple images not yet supported");
        }?;
        let exit_burn_seq = exit_burn.sequence();
        let entry_pos = exit_burn_seq.sequence_pos().first().unwrap();
        let exit_pos = exit_burn_seq.sequence_pos().last().unwrap();
        let entry_t = exit_burn_seq.start_i().t().format("%H:%M:%S").to_string();
        let vel = exit_burn_seq.sequence_vel().last().unwrap();
        let tar = zo.get_imaging_points()[0];
        let det_dt = exit_burn_seq.detumble_dt();
        let acq_dt = exit_burn_seq.acc_dt();
        let tar_unwrap = exit_burn.unwrapped_target();
        info!("Calculated Burn Sequence for Zoned Objective: {}", zo.id());
        log!("Entry at {entry_t}, Position will be {entry_pos}");
        log!("Exit after {acq_dt}s, Position will be {exit_pos}. Detumble time is {det_dt}s.");
        log!("Exit Velocity will be {vel} aiming for target at {tar} unwrapped to {tar_unwrap}.");
        log!("Whole BS: {:?}", exit_burn);
        if let BaseMode::BeaconObjectiveScanningMode = base {
            let burn_start = exit_burn_seq.start_i().t();
            let worst_case_first_comms_end = {
                let to_comms = FlightComputer::get_to_comms_dt_est(context.k().f_cont()).await;
                Utc::now()
                    + to_comms.1
                    + TimeDelta::seconds(TaskController::IN_COMMS_SCHED_SECS as i64)
            };
            if worst_case_first_comms_end + TimeDelta::seconds(10) > burn_start {
                base = BaseMode::MappingMode;
            }
        }
        Some(ZOPrepMode { base, exit_burn, target: zo, left_orbit: AtomicBool::new(false) })
    }

    fn new_base(&self, base: BaseMode) -> Self {
        Self {
            base,
            exit_burn: self.exit_burn.clone(),
            target: self.target.clone(),
            left_orbit: AtomicBool::new(self.left_orbit.load(Ordering::Acquire)),
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
        // TODO: check here if even one transition to comms is possible
        let comms_end = self.base.handle_sched_preconditions(Arc::clone(&context)).await;
        let end = EndCondition::from_burn(self.exit_burn.sequence());
        let sched_handle = {
            let cancel_clone = cancel_task.clone();
            self.base
                .get_schedule_handle(Arc::clone(&context), cancel_clone, comms_end, Some(end))
                .await
        };
        tokio::pin!(sched_handle);
        let safe_mon = context.super_v().safe_mon();
        tokio::select!(
            _ = &mut sched_handle => {
                info!("Additionally scheduling Orbit Escape Burn Sequence!");
                context.k().t_cont().schedule_vel_change(self.exit_burn.sequence().clone(), OrbitEscape).await;
                context.k().con().send_tasklist().await;
            },
            () = safe_mon.notified() => {
                cancel_task.cancel();
                sched_handle.await.ok();
                return self.safe_handler(context).await;
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
                self.left_orbit.store(true, Ordering::Release);
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
        FlightComputer::escape_safe(context.k().f_cont(), false).await;
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

    async fn zo_handler(
        &self,
        context: Arc<ModeContext>,
        obj: KnownImgObjective,
    ) -> Option<OpExitSignal> {
        let burn_dt_cond =
            self.exit_burn.sequence().start_i().t() - Utc::now() > Self::MIN_REPLANNING_DT;
        if obj.end() < self.target.end() && burn_dt_cond {
            let context_clone = Arc::clone(&context);
            let new_obj_mode = Self::from_obj(context_clone, obj.clone(), self.base.clone()).await;
            if let Some(prep_mode) = new_obj_mode {
                context.o_ch_lock().write().await.finish(
                    context.k().f_cont().read().await.current_pos(),
                    self.new_zo_rationale(),
                );
                return Some(OpExitSignal::ReInit(Box::new(prep_mode)));
            }
        }
        context.k_buffer().lock().await.push(obj);
        None
    }

    fn bo_event_handler(&self) -> Option<OpExitSignal> {
        let new_base = self.base.bo_event();
        todo!()
    }

    fn resched_event_handler(&self) -> Option<OpExitSignal> {
        Some(OpExitSignal::ReInit(Box::new(self.clone())))
    }

    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.tasks_done_exit_rationale(),
        );
        if self.left_orbit.load(Ordering::Acquire) {
            Box::new(ZORetrievalMode::new(
                self.target.clone(),
                *self.exit_burn.unwrapped_target(),
            ))
        } else {
            error!("ZOPrepMode::exit_mode called without left_orbit flag set!");
            Box::new(InOrbitMode::new(self.base.clone()))
        }
    }
}
