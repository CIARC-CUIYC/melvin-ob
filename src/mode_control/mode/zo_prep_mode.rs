use crate::flight_control::flight_state::{FlightState, TRANS_DEL};
use crate::flight_control::orbit::{BurnSequence, ExitBurnResult};
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
use crate::mode_control::signal::OptOpExitSignal;
use crate::mode_control::{
    base_mode::BaseMode,
    mode::{
        global_mode::{GlobalMode, OrbitalMode},
        in_orbit_mode::InOrbitMode,
    },
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use crate::{DT_0, error, fatal, info, log};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use std::mem::discriminant;
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
            base: self.base,
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
        context: &Arc<ModeContext>,
        zo: KnownImgObjective,
        curr_base: BaseMode,
    ) -> Option<Self> {
        log!("Trying ZOPrepMode for Zoned Objective: {}", zo.id());
        let due = zo.end();
        let (current_vel, fuel_left) = {
            let f_cont_lock = context.k().f_cont();
            let f_cont = f_cont_lock.read().await;
            (f_cont.current_vel(), f_cont.fuel_left())
        };
        let exit_burn = if zo.min_images() == 1 {
            let target = zo.get_single_image_point();
            TaskController::calculate_single_target_burn_sequence(
                context.o_ch_clone().await.i_entry(),
                current_vel,
                target,
                due,
                fuel_left,
            )
        } else {
            let entries = zo.get_corners();
            TaskController::calculate_multi_target_burn_sequence(
                context.o_ch_clone().await.i_entry(),
                current_vel,
                entries,
                due,
                fuel_left
            )
        }?;
        Self::log_burn(&exit_burn, &zo);
        let base = Self::overthink_base(context, curr_base, exit_burn.sequence()).await;
        Some(ZOPrepMode { base, exit_burn, target: zo, left_orbit: AtomicBool::new(false) })
    }

    fn log_burn(exit_burn: &ExitBurnResult, target: &KnownImgObjective) {
        let exit_burn_seq = exit_burn.sequence();
        let entry_pos = exit_burn_seq.sequence_pos().first().unwrap();
        let exit_pos = exit_burn_seq.sequence_pos().last().unwrap();
        let entry_t = exit_burn_seq.start_i().t().format("%H:%M:%S").to_string();
        let vel = exit_burn_seq.sequence_vel().last().unwrap();
        let tar = exit_burn.target_pos();
        let add_tar = exit_burn.add_target();
        let det_dt = exit_burn_seq.detumble_dt();
        let acq_dt = exit_burn_seq.acc_dt();
        let tar_unwrap = exit_burn.unwrapped_target();
        info!(
            "Calculated Burn Sequence for Zoned Objective: {}",
            target.id()
        );
        log!("Entry at {entry_t}, Position will be {entry_pos}");
        log!("Exit after {acq_dt}s, Position will be {exit_pos}. Detumble time is {det_dt}s.");
        log!("Exit Velocity will be {vel} aiming for target at {tar} unwrapped to {tar_unwrap}.");
        if let Some(tar2) = add_tar {
            log!("Additional Target will be {tar2}");
        }
        log!("Whole BS: {:?}", exit_burn);
    }

    fn new_base(&self, base: BaseMode) -> Self {
        Self {
            base,
            exit_burn: self.exit_burn.clone(),
            target: self.target.clone(),
            left_orbit: AtomicBool::new(self.left_orbit.load(Ordering::Acquire)),
        }
    }

    #[allow(clippy::cast_possible_wrap)]
    async fn overthink_base(c: &Arc<ModeContext>, base: BaseMode, burn: &BurnSequence) -> BaseMode {
        if matches!(base, BaseMode::MappingMode) {
            return BaseMode::MappingMode;
        }
        let burn_start = burn.start_i().t();
        let worst_case_first_comms_end = {
            let to_dt = FlightComputer::get_to_comms_dt_est(c.k().f_cont()).await;
            let state_change = (FlightState::Comms, FlightState::Acquisition);
            let from_dt = TimeDelta::from_std(TRANS_DEL[&state_change]).unwrap_or(DT_0);
            to_dt + TimeDelta::seconds(TaskController::IN_COMMS_SCHED_SECS as i64) + from_dt
        };
        if worst_case_first_comms_end + TimeDelta::seconds(5) > burn_start {
            let t = worst_case_first_comms_end.format("%d %H:%M:%S").to_string();
            log!("Requested BOScanningMode not feasible, first comms end is {t}.");
            BaseMode::MappingMode
        } else {
            BaseMode::BeaconObjectiveScanningMode
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
        let new_base = Self::overthink_base(&context, self.base, self.exit_burn.sequence()).await;
        if discriminant(&self.base) != discriminant(&new_base) {
            return OpExitSignal::ReInit(Box::new(self.new_base(new_base)));
        }
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

    async fn exec_task_wait(&self, c: Arc<ModeContext>, due: DateTime<Utc>) -> WaitExitSignal {
        <Self as OrbitalMode>::exec_task_wait(self, c, due).await
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
        let new = Self::from_obj(&context, self.target.clone(), self.base).await;
        OpExitSignal::ReInit(new.map_or(Box::new(InOrbitMode::new(self.base)), |b| Box::new(b)))
    }

    async fn zo_handler(&self, c: &Arc<ModeContext>, obj: KnownImgObjective) -> OptOpExitSignal {
        let burn_dt_cond =
            self.exit_burn.sequence().start_i().t() - Utc::now() > Self::MIN_REPLANNING_DT;
        if obj.end() < self.target.end() && burn_dt_cond {
            let new_obj_mode = Self::from_obj(c, obj.clone(), self.base).await;
            if let Some(prep_mode) = new_obj_mode {
                c.o_ch_lock().write().await.finish(
                    c.k().f_cont().read().await.current_pos(),
                    self.new_zo_rationale(),
                );
                return Some(OpExitSignal::ReInit(Box::new(prep_mode)));
            }
        }
        c.k_buffer().lock().await.push(obj);
        None
    }

    async fn bo_event_handler(&self, context: &Arc<ModeContext>) -> OptOpExitSignal {
        let prop_new_base = self.base.bo_event();
        let new_base = Self::overthink_base(context, prop_new_base, self.exit_burn.sequence()).await;
        if discriminant(&self.base) == discriminant(&new_base) {
            None
        } else {
            self.log_bo_event(context, new_base).await;
            log!("Trying to change base mode from {} to {} due to BO Event!", self.base, new_base);
            Some(OpExitSignal::ReInit(Box::new(self.new_base(new_base))))
        }
    }

    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.tasks_done_exit_rationale(),
        );
        if self.left_orbit.load(Ordering::Acquire) {
            Box::new(ZORetrievalMode::new(
                self.target.clone(),
                self.exit_burn.add_target(),
                *self.exit_burn.unwrapped_target(),
            ))
        } else {
            error!("ZOPrepMode::exit_mode called without left_orbit flag set!");
            Box::new(InOrbitMode::new(self.base))
        }
    }
}
