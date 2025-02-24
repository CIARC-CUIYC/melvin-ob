use crate::flight_control::{
    flight_computer::FlightComputer,
    flight_state::FlightState,
    objective::{
        beacon_objective::BeaconObjective, known_img_objective::KnownImgObjective,
        objective_base::ObjectiveBase, objective_type::ObjectiveType,
    },
    task::{
        base_task::{BaseTask, Task},
        TaskController,
    },
};
use crate::mode_control::{
    base_mode::{BaseMode, BaseWaitExitSignal, TaskEndSignal::Join},
    mode::global_mode::{ExecExitSignal, GlobalMode, OpExitSignal, WaitExitSignal},
    mode_context::ModeContext,
};
use crate::{fatal, info, log, obj, warn};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use std::{future::Future, pin::Pin, sync::Arc, time::Duration};
use tokio::task::JoinError;
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub struct InOrbitMode {
    base: BaseMode,
}

impl InOrbitMode {
    const MODE_NAME: &'static str = "InOrbitMode";
    const MAX_WAIT_TIMEDELTA: TimeDelta = TimeDelta::seconds(10);

    pub fn new() -> Self {
        Self {
            base: BaseMode::MappingMode,
        }
    }

    pub async fn sched_and_map(&self, context: Arc<ModeContext>, c_tok: CancellationToken) {
        let k_clone = Arc::clone(context.k());

        let j_handle = {
            let orbit_char = context.o_ch_clone().await;
            tokio::spawn(async move {
                TaskController::sched_opt_orbit(
                    k_clone.t_cont(),
                    k_clone.c_orbit(),
                    k_clone.f_cont(),
                    orbit_char.i_entry(),
                )
                .await;
            })
        };
        let state = context.k().f_cont().read().await.state();
        if state == FlightState::Acquisition {
            BaseMode::exec_map(context, Join(j_handle), c_tok).await;
        } else {
            j_handle.await.ok();
        }
    }
}

#[async_trait]
impl GlobalMode for InOrbitMode {
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let cancel_task = CancellationToken::new();
        self.base.handle_sched_preconditions(Arc::clone(&context)).await;
        let sched_handle = {
            let cancel_clone = cancel_task.clone();
            self.base.get_schedule_handle(Arc::clone(&context), cancel_clone).await
        };
        tokio::pin!(sched_handle);
        let safe_mon = context.super_v().safe_mon();
        tokio::select!(
            _ = &mut sched_handle => {
                context.k().con().send_tasklist().await;
            },
            () = safe_mon.notified() => {
                cancel_task.cancel();
                sched_handle.await.ok();

                // Return to mapping mode
                return OpExitSignal::ReInit(Box::new(self.clone()))
            }
        );
        OpExitSignal::Continue
    }

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
            };
            tasks += 1;
        }
        OpExitSignal::Continue
    }

    async fn exec_task_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
    ) -> WaitExitSignal {
        let safe_mon = context.super_v().safe_mon();
        let mut obj_mon = context.obj_mon().write().await;
        let cancel_task = CancellationToken::new();
        let fut: Pin<Box<dyn Future<Output = Result<BaseWaitExitSignal, JoinError>> + Send>> =
            if (due - Utc::now()) > Self::MAX_WAIT_TIMEDELTA {
                Box::pin(self.base.get_wait(Arc::clone(&context), due, cancel_task.clone()).await)
            } else {
                warn!("Task wait time too short. Just waiting!");
                Box::pin(async {
                    tokio::time::sleep(
                        (due - Utc::now()).to_std().unwrap_or(Duration::from_secs(0)),
                    )
                    .await;
                    Ok(BaseWaitExitSignal::Continue)
                })
            };
        tokio::select! {
            exit_sig = fut => {
                let sig = exit_sig.expect("[FATAL] Task wait hung up!");
                match sig {
                    BaseWaitExitSignal::Continue => WaitExitSignal::Continue,
                    sig => WaitExitSignal::BODoneEvent(sig)
                }
            },
            () = safe_mon.notified() => {
                cancel_task.cancel();
                WaitExitSignal::SafeEvent
            },
            obj = async {
                // TODO: later we shouldn't block zoned objectives anymore
                while let Some(msg) = obj_mon.recv().await {
                    match msg.obj_type() {
                        ObjectiveType::Beacon { .. } => {
                            return WaitExitSignal::NewObjectiveEvent(msg);
                        },
                        ObjectiveType::KnownImage { .. } => {
                            continue;
                        }
                    }
                }
                fatal!("Objective monitor hung up!")
                } => {
                cancel_task.cancel();
                obj
            }
        }
    }

    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal {
        match task.task_type() {
            BaseTask::SwitchState(switch) => self.base.get_task(context, *switch).await,
            _ => {
                fatal!(
                    "Illegal task type {} for state {}!",
                    task.task_type(),
                    Self::MODE_NAME
                )
            }
        }
        ExecExitSignal::Continue
    }

    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        FlightComputer::escape_safe(context.k().f_cont()).await;
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.safe_mode_rationale(),
        );
        OpExitSignal::ReInit(Box::new(self.clone()))
    }

    async fn objective_handler(
        &self,
        context: Arc<ModeContext>,
        obj: ObjectiveBase,
    ) -> Option<OpExitSignal> {
        match obj.obj_type() {
            ObjectiveType::Beacon { .. } => {
                let b_obj = BeaconObjective::new(
                    obj.id(),
                    String::from(obj.name()),
                    obj.start(),
                    obj.end(),
                );
                obj!("Found new Beacon Objective {}!", obj.id());
                if let Some(base) = self.base.handle_b_o(&context, b_obj).await {
                    context.o_ch_lock().write().await.finish(
                        context.k().f_cont().read().await.current_pos(),
                        self.new_bo_rationale(),
                    );
                    Some(OpExitSignal::ReInit(Box::new(Self { base })))
                } else {
                    None
                }
            }
            ObjectiveType::KnownImage {
                zone,
                optic_required,
                coverage_required,
            } => {
                obj!("Found new Zoned Objective {}!", obj.id());
                let k_obj = KnownImgObjective::new(
                    obj.id(),
                    String::from(obj.name()),
                    obj.start(),
                    obj.end(),
                    *zone,
                    *optic_required,
                    *coverage_required,
                );
                context.o_ch_lock().write().await.finish(
                    context.k().f_cont().read().await.current_pos(),
                    self.new_zo_rationale(),
                );
                // TODO: return ZonedObjectivePrepMode
                None
                //OpExitSignal::ReInit(Box::New())
            }
        }
    }

    async fn b_o_done_handler(
        &self,
        context: Arc<ModeContext>,
        b_sig: BaseWaitExitSignal,
    ) -> OpExitSignal {
        match b_sig {
            BaseWaitExitSignal::Continue => OpExitSignal::Continue,
            BaseWaitExitSignal::ReturnAllDone => {
                context.o_ch_lock().write().await.finish(
                    context.k().f_cont().read().await.current_pos(),
                    self.bo_done_rationale(),
                );
                OpExitSignal::ReInit(Box::new(Self {
                    base: BaseMode::MappingMode,
                }))
            }
            BaseWaitExitSignal::ReturnSomeLeft => {
                context.o_ch_lock().write().await.finish(
                    context.k().f_cont().read().await.current_pos(),
                    self.bo_left_rationale(),
                );
                OpExitSignal::ReInit(Box::new(self.clone()))
            }
        }
    }

    async fn exit_mode(&self, c: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        let handler = c.k().f_cont().read().await.client();
        Box::new(Self {
            base: self.base.exit_base(handler).await,
        })
    }
}
