use async_trait::async_trait;
use std::sync::Arc;
use crate::flight_control::task::base_task::Task;
use crate::mode_control::mode_context::StateContext;

#[async_trait]
pub trait GlobalMode {
    fn safe_mode_rationale(&self) -> &'static str { "SAFE mode not specified!" }
    fn new_zo_rationale(&self) -> &'static str { "newly discovered ZO!" }
    fn new_bo_rationale(&self) -> &'static str { "newly discovered BO!" }
    fn tasks_done_rationale(&self) -> &'static str { "tasks list done!" }
    fn type_name(&self) -> &'static str;
    async fn init_mode(&self, context: Arc<StateContext>) -> OpExitSignal;
    async fn exec_task_queue(&self, context: Arc<StateContext>) -> OpExitSignal;
    async fn exec_task_wait(&self, context: Arc<StateContext>, due_time: chrono::TimeDelta) -> ExecExitSignal;
    async fn exec_task(&self, context: Arc<StateContext>, task: Task) -> ExecExitSignal;
    async fn safe_handler(&self, context: Arc<StateContext>) -> OpExitSignal;
    async fn exit_mode(&self, context: Arc<StateContext>) -> Box<dyn GlobalMode>;
}

pub enum OpExitSignal {
    ReInit(Box<dyn GlobalMode>),
    Continue,
}

pub enum ExecExitSignal {
    Continue,
    SafeEvent,
    
}