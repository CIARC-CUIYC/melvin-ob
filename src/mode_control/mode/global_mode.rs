use crate::flight_control::{
    objective::objective_base::ObjectiveBase,
    task::base_task::Task,
};
use crate::mode_control::{
    base_mode::{BaseMode, BaseWaitExitSignal},
    mode_context::ModeContext,
    signal::{WaitExitSignal, ExecExitSignal, OpExitSignal},
};
use crate::{fatal, info, log, warn};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use std::{future::Future, pin::Pin, sync::Arc, time::Duration};
use tokio::task::JoinError;
use tokio_util::sync::CancellationToken;

#[async_trait]
pub trait GlobalMode: Sync {
    fn safe_mode_rationale(&self) -> &'static str { "SAFE mode Event!" }
    fn new_zo_rationale(&self) -> &'static str { "newly discovered ZO!" }
    fn new_bo_rationale(&self) -> &'static str { "newly discovered BO!" }
    fn tasks_done_rationale(&self) -> &'static str { "tasks list done!" }
    fn bo_done_rationale(&self) -> &'static str { "BO done or expired!" }
    fn bo_left_rationale(&self) -> &'static str { "necessary Rescheduling for remaining BOs!" }
    fn type_name(&self) -> &'static str;
    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal;

    #[allow(clippy::cast_sign_loss, clippy::cast_precision_loss)]
    async fn exec_task_queue(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let context_local = Arc::clone(&context);
        let mut tasks = 0;
        while let Some(task) = {
            let sched_arc = context_local.k().t_cont().sched_arc();
            let mut sched_lock = sched_arc.write().await;
            let t = sched_lock.pop_front();
            drop(sched_lock);
            t
        } {
            let due_time = task.dt() - Utc::now();
            let task_type = task.task_type();
            info!("TASK {tasks}: {task_type} in  {}s!", due_time.num_seconds());
            while task.dt() > Utc::now() + TimeDelta::seconds(2) {
                let context_clone = Arc::clone(&context);
                match self.exec_task_wait(context_clone, task.dt()).await {
                    WaitExitSignal::Continue => {}
                    WaitExitSignal::SafeEvent => {
                        return self.safe_handler(context_local).await;
                    }
                    WaitExitSignal::NewObjectiveEvent(obj) => {
                        let context_clone = Arc::clone(&context);
                        let ret = self.objective_handler(context_clone, obj).await;
                        if let Some(opt) = ret {
                            return opt;
                        };
                    }
                    WaitExitSignal::BODoneEvent(sig) => {
                        return self.b_o_done_handler(context, sig).await
                    }
                };
            }
            let task_delay = (task.dt() - Utc::now()).num_milliseconds() as f32 / 1000.0;
            if task_delay.abs() > 2.0 {
                log!("Task {tasks} delayed by {task_delay}s!");
            }
            let context_clone = Arc::clone(&context);
            match self.exec_task(context_clone, task).await {
                ExecExitSignal::Continue => {}
                ExecExitSignal::SafeEvent => {
                    return self.safe_handler(context_local).await;
                }
                ExecExitSignal::NewObjectiveEvent(_) => {
                    fatal!("Unexpected task exit signal!");
                }
                ExecExitSignal::ExitedOrbit(zo) => {
                    todo!()
                }
            };
            tasks += 1;
        }
        OpExitSignal::Continue
    }
    async fn exec_task_wait(&self, context: Arc<ModeContext>, due: DateTime<Utc>)
        -> WaitExitSignal;
    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal;
    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal;
    async fn objective_handler(
        &self,
        context: Arc<ModeContext>,
        obj: ObjectiveBase,
    ) -> Option<OpExitSignal>;
    async fn b_o_done_handler(
        &self,
        context: Arc<ModeContext>,
        b_sig: BaseWaitExitSignal,
    ) -> OpExitSignal;
    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode>;
}

pub trait OrbitalMode: GlobalMode {
    fn get_max_dt() -> TimeDelta { TimeDelta::seconds(10) }
    
    fn base(&self) -> &BaseMode;

    async fn exec_task_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
    ) -> WaitExitSignal {
        let safe_mon = context.super_v().safe_mon();
        let mut obj_mon = context.obj_mon().write().await;
        let cancel_task = CancellationToken::new();
        let fut: Pin<Box<dyn Future<Output = Result<BaseWaitExitSignal, JoinError>> + Send>> =
            if (due - Utc::now()) > Self::get_max_dt() {
                Box::pin(self.base().get_wait(Arc::clone(&context), due, cancel_task.clone()).await)
            } else {
                warn!("Task wait time too short. Just waiting!");
                Box::pin(async {
                    let sleep = (due - Utc::now()).to_std().unwrap_or(Duration::from_secs(0));
                    tokio::time::timeout(sleep, cancel_task.cancelled()).await.ok().unwrap_or(());
                    Ok(BaseWaitExitSignal::Continue)
                })
            };
        tokio::pin!(fut);
        tokio::select! {
            exit_sig = &mut fut => {
                let sig = exit_sig.expect("[FATAL] Task wait hung up!");
                match sig {
                    BaseWaitExitSignal::Continue => WaitExitSignal::Continue,
                    signal => WaitExitSignal::BODoneEvent(signal)
                }
            },
            () = safe_mon.notified() => {
                cancel_task.cancel();
                fut.await.ok();
                WaitExitSignal::SafeEvent
            },
            msg =  obj_mon.recv() => {
                let obj = msg.expect("[FATAL] Objective monitor hung up!");
                cancel_task.cancel();
                fut.await.ok();
                WaitExitSignal::NewObjectiveEvent(obj)
            }
        }
    }
}
