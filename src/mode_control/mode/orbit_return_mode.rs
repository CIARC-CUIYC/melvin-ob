use crate::flight_control::{objective::objective_base::ObjectiveBase, task::base_task::Task};
use crate::mode_control::{
    base_mode::BaseWaitExitSignal,
    mode::global_mode::GlobalMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use std::sync::Arc;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::task::TaskController;

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
        TaskController::schedule_orbit_return();
        todo!() 
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

    async fn objective_handler(
        &self,
        context: Arc<ModeContext>,
        obj: ObjectiveBase,
    ) -> Option<OpExitSignal> {
        todo!()
    }

    async fn b_o_done_handler(
        &self,
        context: Arc<ModeContext>,
        b_sig: BaseWaitExitSignal,
    ) -> OpExitSignal {
        todo!()
    }

    async fn exit_mode(&self, c: Arc<ModeContext>) -> Box<dyn GlobalMode> { todo!() }
}
