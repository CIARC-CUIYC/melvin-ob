use crate::flight_control::task::TaskController;
use crate::flight_control::{
    beacon_controller::BeaconControllerState, flight_computer::FlightComputer,
    objective::known_img_objective::KnownImgObjective, task::base_task::Task,
};
use crate::mode_control::signal::OptOpExitSignal;
use crate::mode_control::{
    base_mode::BaseMode,
    mode::{global_mode::GlobalMode, in_orbit_mode::InOrbitMode, zo_prep_mode::ZOPrepMode},
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use std::sync::Arc;
use crate::obj;

#[derive(Clone)]
pub struct OrbitReturnMode {}

impl OrbitReturnMode {
    const MODE_NAME: &'static str = "OrbitReturnMode";

    pub fn new() -> Self { Self {} }

    async fn get_next_mode(context: &Arc<ModeContext>) -> Box<dyn GlobalMode> {
        let next_base_mode = Self::get_next_base_mode(context).await;
        let mut obj_mon = context.zo_mon().write().await;
        let mut k_buffer = context.k_buffer().lock().await;
        while let Ok(next_obj) = obj_mon.try_recv() {
            k_buffer.push(next_obj);
        };        
        k_buffer.retain(|obj| Utc::now() < obj.end());
        while let Some(obj) = k_buffer.pop() {
            let res = ZOPrepMode::from_obj(context, obj, next_base_mode).await;
            if let Some(prep_mode) = res {
                return Box::new(prep_mode);
            }
        }
        Box::new(InOrbitMode::new(next_base_mode))
    }

    async fn get_next_base_mode(context: &Arc<ModeContext>) -> BaseMode {
        let beacon_cont_state = {
            let mut bo_mon = context.bo_mon().write().await;
            *bo_mon.borrow_and_update()
        };
        match beacon_cont_state {
            BeaconControllerState::ActiveBeacons => BaseMode::BeaconObjectiveScanningMode,
            BeaconControllerState::NoActiveBeacons => BaseMode::MappingMode,
        }
    }
}

#[async_trait]
impl GlobalMode for OrbitReturnMode {
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let safe_mon = context.super_v().safe_mon();
        let f_cont_clone = context.k().f_cont().clone();
        let fut = async {
            FlightComputer::get_to_static_orbit_vel(&f_cont_clone).await;
            let max_maneuver_batt = FlightComputer::max_or_maneuver_charge();
            let batt = f_cont_clone.read().await.current_battery();
            if batt < max_maneuver_batt {
                FlightComputer::charge_to_wait(&f_cont_clone, max_maneuver_batt).await;
            }
            FlightComputer::or_maneuver(context.k().f_cont(), context.k().c_orbit()).await
        };
        tokio::select! {
        new_i = fut => {
                let pos = context.k().f_cont().read().await.current_pos();
                context.o_ch_lock().write().await.finish_entry(pos, new_i);
                OpExitSignal::ReInit(self.exit_mode(context).await)
            },
        () = safe_mon.notified() => self.safe_handler(context).await
        }
    }

    async fn exec_task_wait(&self, _: Arc<ModeContext>, _: DateTime<Utc>) -> WaitExitSignal {
        unimplemented!()
    }

    async fn exec_task(&self, _: Arc<ModeContext>, _: Task) -> ExecExitSignal { unimplemented!() }

    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        FlightComputer::escape_safe(context.k().f_cont(), false).await;
        OpExitSignal::ReInit(Box::new(OrbitReturnMode::new()))
    }

    async fn zo_handler(&self, c: &Arc<ModeContext>, obj: KnownImgObjective) -> OptOpExitSignal {
        obj!("Found new Zoned Objective with ID: {} in mode {}.Stashing!", obj.id(), Self::MODE_NAME);
        c.k_buffer().lock().await.push(obj);
        None
    }

    async fn bo_event_handler(&self, _: &Arc<ModeContext>) -> OptOpExitSignal { unimplemented!() }

    fn resched_event_handler(&self) -> OptOpExitSignal { unimplemented!() }

    async fn exit_mode(&self, c: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        if c.k().f_cont().read().await.current_battery() < TaskController::MIN_BATTERY_THRESHOLD {
            FlightComputer::charge_to_wait(&c.k().f_cont(), TaskController::MIN_BATTERY_THRESHOLD)
                .await;
        }
        Self::get_next_mode(&c).await
    }
}
