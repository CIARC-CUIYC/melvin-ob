use crate::flight_control::{
    objective::{
        known_img_objective::KnownImgObjective,
        objective_base::ObjectiveBase,
    },
    task::base_task::Task,
};
use crate::mode_control::{
    base_mode::BaseWaitExitSignal,
    mode::global_mode::GlobalMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use std::sync::Arc;

#[derive(Clone)]
pub struct ZORetrievalMode {
    target: KnownImgObjective,
}

impl ZORetrievalMode {
    const MODE_NAME: &'static str = "ZORetrievalMode";
    pub fn new(target: KnownImgObjective) -> Self {
        Self { target }
    }
}

#[async_trait]
impl GlobalMode for ZORetrievalMode {
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        unimplemented!()
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

    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        unimplemented!()
    }

    async fn objective_handler(
        &self,
        context: Arc<ModeContext>,
        obj: ObjectiveBase,
    ) -> Option<OpExitSignal> {
        unimplemented!()
    }

    async fn b_o_done_handler(
        &self,
        context: Arc<ModeContext>,
        b_sig: BaseWaitExitSignal,
    ) -> OpExitSignal {
        unimplemented!()
    }

    async fn exit_mode(&self, c: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        unimplemented!()
    }
}
