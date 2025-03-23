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
        FlightComputer::get_to_static_orbit_vel(context.k().f_cont()).await;
        todo!()
        //TaskController::schedule_orbit_return();
    }

    async fn exec_task_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
    ) -> WaitExitSignal {
        todo!()
    }

    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal { todo!() }

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
