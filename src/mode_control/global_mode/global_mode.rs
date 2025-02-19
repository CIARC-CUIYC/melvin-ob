use async_trait::async_trait;
use std::sync::Arc;
use chrono::{DateTime, Utc};
use crate::flight_control::objective::objective_base::ObjectiveBase;
use crate::flight_control::task::base_task::Task;
use crate::mode_control::mode_context::ModeContext;

#[async_trait]
pub trait GlobalMode {
    fn safe_mode_rationale(&self) -> &'static str { "SAFE mode Event!" }
    fn new_zo_rationale(&self) -> &'static str { "newly discovered ZO!" }
    fn new_bo_rationale(&self) -> &'static str { "newly discovered BO!" }
    fn tasks_done_rationale(&self) -> &'static str { "tasks list done!" }
    fn type_name(&self) -> &'static str;
    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal;
    async fn exec_task_queue(&self, context: Arc<ModeContext>) -> OpExitSignal;
    async fn exec_task_wait(&self, context: Arc<ModeContext>, due: DateTime<Utc>) -> ExecExitSignal;
    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal;
    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal;
    async fn objective_handler(&self, context: Arc<ModeContext>, obj: ObjectiveBase) -> Option<OpExitSignal>;
    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode>;
}

pub enum OpExitSignal {
    ReInit(Box<dyn GlobalMode>),
    Continue,
}

pub enum ExecExitSignal {
    Continue,
    SafeEvent,
    NewObjectiveEvent(ObjectiveBase),
    
}