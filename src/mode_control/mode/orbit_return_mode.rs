use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::objective::known_img_objective::KnownImgObjective;
use crate::flight_control::task::base_task::Task;
use crate::mode_control::{
    mode::global_mode::GlobalMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use std::sync::Arc;

#[derive(Clone)]
pub struct OrbitReturnMode {}

impl OrbitReturnMode {
    const MODE_NAME: &'static str = "OrbitReturnMode";

    pub fn new() -> Self { Self {} }
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
        _ = safe_mon.notified() => self.safe_handler(context).await
        }
    }

    async fn exec_task_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
    ) -> WaitExitSignal {
        unimplemented!()
    }

    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal {
        unimplemented!()
    }

    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal { todo!() }

    async fn zo_handler(
        &self,
        context: Arc<ModeContext>,
        obj: KnownImgObjective,
    ) -> Option<OpExitSignal> {
        context.k_buffer().lock().await.push(obj);
        None
    }

    fn bo_event_handler(&self) -> Option<OpExitSignal> { unimplemented!() }

    fn resched_event_handler(&self) -> Option<OpExitSignal> { unimplemented!() }

    async fn exit_mode(&self, c: Arc<ModeContext>) -> Box<dyn GlobalMode> { todo!() }
}
