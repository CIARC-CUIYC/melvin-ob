use crate::flight_control::task::base_task::Task;
use crate::mode_control::{
    base_mode::BaseMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use crate::{DT_0_STD, fatal, info, log, warn};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use std::{future::Future, pin::Pin, sync::Arc};
use std::mem::discriminant;
use tokio::sync::RwLock;
use tokio::sync::watch::Receiver;
use tokio::task::JoinError;
use tokio_util::sync::CancellationToken;
use crate::flight_control::beacon_controller::BeaconControllerState;
use crate::flight_control::objective::known_img_objective::KnownImgObjective;
use crate::mode_control::signal::BaseWaitExitSignal;

#[async_trait]
pub trait GlobalMode: Sync {
    fn safe_mode_rationale(&self) -> &'static str { "SAFE mode Event!" }
    fn new_zo_rationale(&self) -> &'static str { "newly discovered ZO!" }
    fn new_bo_rationale(&self) -> &'static str { "newly discovered BO!" }
    fn tasks_done_rationale(&self) -> &'static str { "tasks list done!" }
    fn tasks_done_exit_rationale(&self) -> &'static str {
        "tasks list done and exited orbit for ZO Retrieval!"
    }
    fn out_of_orbit_rationale(&self) -> &'static str { "out of orbit without purpose!" }
    fn bo_done_rationale(&self) -> &'static str { "BO done or expired!" }
    fn bo_left_rationale(&self) -> &'static str { "necessary rescheduling for remaining BOs!" }
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
            let due_time = task.t() - Utc::now();
            let task_type = task.task_type();
            info!("TASK {tasks}: {task_type} in  {}s!", due_time.num_seconds());
            while task.t() > Utc::now() + TimeDelta::seconds(2) {
                let context_clone = Arc::clone(&context);
                match self.exec_task_wait(context_clone, task.t()).await {
                    WaitExitSignal::Continue => {}
                    WaitExitSignal::SafeEvent => {
                        return self.safe_handler(context_local).await;
                    }
                    WaitExitSignal::NewZOEvent(obj) => {
                        let context_clone = Arc::clone(&context);
                        if let Some(opt) = self.zo_handler(context_clone, obj).await {
                            return opt;
                        };
                    }
                    WaitExitSignal::BOEvent => {
                        if let Some(opt) = self.bo_event_handler() {
                            return opt;
                        };
                    }
                    WaitExitSignal::RescheduleEvent => {
                        if let Some(opt) = self.resched_event_handler() {
                            return opt;
                        }
                    }
                };
            }
            let task_delay = (task.t() - Utc::now()).num_milliseconds() as f32 / 1000.0;
            if task_delay.abs() > 2.0 {
                log!("Task {tasks} delayed by {task_delay}s!");
            }
            let context_clone = Arc::clone(&context);
            match self.exec_task(context_clone, task).await {
                ExecExitSignal::Continue => {}
                ExecExitSignal::SafeEvent => {
                    return self.safe_handler(context_local).await;
                }
                ExecExitSignal::NewZOEvent(_) => {
                    fatal!("Unexpected task exit signal!");
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
    async fn zo_handler(&self, context: Arc<ModeContext>, obj: KnownImgObjective) -> Option<OpExitSignal>;
    fn bo_event_handler(&self) -> Option<OpExitSignal>;
    fn resched_event_handler(&self) -> Option<OpExitSignal>;
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
        let mut zo_mon = context.zo_mon().write().await;
        let bo_mon = context.bo_mon();
        let cancel_task = CancellationToken::new();

        let fut: Pin<Box<dyn Future<Output = Result<BaseWaitExitSignal, JoinError>> + Send>> =
            if (due - Utc::now()) > Self::get_max_dt() {
                Box::pin(self.base().get_wait(Arc::clone(&context), due, cancel_task.clone()).await)
            } else {
                warn!("Task wait time too short. Just waiting!");
                Box::pin(async {
                    let sleep = (due - Utc::now()).to_std().unwrap_or(DT_0_STD);
                    tokio::time::timeout(sleep, cancel_task.cancelled()).await.ok().unwrap_or(());
                    Ok(BaseWaitExitSignal::Continue)
                })
            };
        let bo_change_signal = self.base().get_rel_bo_event();
        tokio::pin!(fut);
        tokio::select! {
            exit_sig = &mut fut => {
                let sig = exit_sig.expect("[FATAL] Task wait hung up!");
                match sig {
                    BaseWaitExitSignal::Continue => WaitExitSignal::Continue,
                    BaseWaitExitSignal::ReSchedule => WaitExitSignal::RescheduleEvent
                }
            },
            () = safe_mon.notified() => {
                cancel_task.cancel();
                fut.await.ok();
                WaitExitSignal::SafeEvent
            },
            msg =  zo_mon.recv() => {
                let img_obj = msg.expect("[FATAL] Objective monitor hung up!");
                cancel_task.cancel();
                fut.await.ok();
                WaitExitSignal::NewZOEvent(img_obj)
            }
            () = Self::monitor_bo_mon_change(bo_change_signal, bo_mon) => {
                cancel_task.cancel();
                fut.await.ok();
                WaitExitSignal::BOEvent            
            }
            
        }
    }
    
    async fn monitor_bo_mon_change(sig: BeaconControllerState, bo_mon: &RwLock<Receiver<BeaconControllerState>>) {
        let mut bo_mon_lock = bo_mon.write().await;
        loop {
            if let Ok(()) = bo_mon_lock.changed().await {
                let sent_sig = bo_mon_lock.borrow_and_update();
                let sent_sig_ref = &*sent_sig;
                if discriminant(&sig) == discriminant(sent_sig_ref) {
                    return;
                }
            }
            
        }
    }
}
