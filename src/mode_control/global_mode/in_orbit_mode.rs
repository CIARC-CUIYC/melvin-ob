use crate::flight_control::objective::beacon_objective::BeaconObjective;
use crate::flight_control::objective::objective_base::ObjectiveBase;
use crate::flight_control::objective::objective_type::ObjectiveType;
use crate::flight_control::{
    flight_computer::FlightComputer,
    flight_state::FlightState,
    task::{
        base_task::{BaseTask, Task},
        TaskController,
    },
};
use crate::mode_control::{
    base_mode::{BaseMode, MappingModeEnd::Join},
    global_mode::global_mode::{ExecExitSignal, GlobalMode, OpExitSignal},
    mode_context::StateContext,
};
use async_trait::async_trait;
use std::{future::Future, pin::Pin, sync::Arc};
use tokio::task::JoinError;
use tokio_util::sync::CancellationToken;
use crate::flight_control::objective::known_img_objective::KnownImgObjective;

#[derive(Clone)]
pub struct InOrbitMode {
    base: BaseMode,
}

impl InOrbitMode {
    const MODE_NAME: &'static str = "InOrbitMode";
    const MAX_WAIT_DURATION: chrono::TimeDelta = chrono::TimeDelta::seconds(10);

    pub fn new() -> Self {
        Self {
            base: BaseMode::MappingMode,
        }
    }

    pub async fn sched_and_map(context: Arc<StateContext>, c_tok: CancellationToken) {
        let j_handle = {
            // TODO: self.base.get_schedule_handle();
            let k_clone_clone = Arc::clone(context.k());
            let orbit_char = context.o_ch_clone().await;
            tokio::spawn(async move {
                TaskController::schedule_optimal_orbit(
                    k_clone_clone.t_cont(),
                    k_clone_clone.c_orbit(),
                    k_clone_clone.f_cont(),
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

    async fn init_mode(&self, context: Arc<StateContext>) -> OpExitSignal {
        let cancel_task = CancellationToken::new();
        let sched_handle = {
            let cancel_clone = cancel_task.clone();
            let context_clone = Arc::clone(&context);
            tokio::spawn(async move {
                Self::sched_and_map(context_clone, cancel_clone).await;
            })
        };
        tokio::pin!(sched_handle);
        tokio::select!(
            _ = &mut sched_handle => {
                context.k().con().send_tasklist().await;
            },
            () = context.safe_mon().notified() => {
                cancel_task.cancel();
                sched_handle.abort();

                // Return to mapping mode
                return OpExitSignal::ReInit(Box::new(self.clone()))
            }
        );
        OpExitSignal::Continue
    }

    async fn exec_task_queue(&self, context: Arc<StateContext>) -> OpExitSignal {
        let context_local = Arc::clone(&context);
        while let Some(task) = {
            let sched_arc = context_local.k().t_cont().sched_arc();
            let mut sched_lock = sched_arc.write().await;
            let t = sched_lock.pop_front();
            drop(sched_lock);
            t
        } {
            let due_time = task.dt() - chrono::Utc::now();
            let phases = context_local.o_ch_clone().await.mode_switches();
            let task_type = task.task_type();
            println!(
                "[INFO] Iteration {phases}: {task_type} in  {}s!",
                due_time.num_seconds()
            );
            let context_clone = Arc::clone(&context);
            match self.exec_task_wait(context_clone, due_time).await {
                ExecExitSignal::Continue => {}
                ExecExitSignal::SafeEvent => {
                    return self.safe_handler(context_local).await;
                }
                ExecExitSignal::NewObjectiveEvent(obj) => {
                    let context_clone = Arc::clone(&context);
                    let ret = self.objective_handler(context_clone, obj).await; 
                    if let Some(opt) = ret { return opt };
                }
            };
            let context_clone = Arc::clone(&context);
            match self.exec_task(context_clone, task).await {
                ExecExitSignal::Continue => {}
                ExecExitSignal::SafeEvent => {
                    return self.safe_handler(context_local).await;
                }
                ExecExitSignal::NewObjectiveEvent(_) => {
                    panic!("[FATAL] Unexpected task exit signal!");
                }
            };
        }
        OpExitSignal::Continue
    }

    async fn exec_task_wait(
        &self,
        context: Arc<StateContext>,
        due_time: chrono::TimeDelta,
    ) -> ExecExitSignal {
        let safe_mon = context.safe_mon();
        let mut obj_mon = context.obj_mon().write().await;
        let cancel_task = CancellationToken::new();
        let fut: Pin<Box<dyn Future<Output = Result<(), JoinError>> + Send>> = if due_time
            > Self::MAX_WAIT_DURATION
        {
            Box::pin(self.base.get_wait(Arc::clone(&context), due_time, cancel_task.clone()).await)
        } else {
            println!("[WARN] Task wait time too short. Just waiting!");
            Box::pin(async {
                tokio::time::sleep(due_time.to_std().unwrap()).await;
                Ok(())
            })
        };
        tokio::select! {
                _ = fut => {
                    ExecExitSignal::Continue
                },
                () = safe_mon.notified() => {
                        cancel_task.cancel();
                        ExecExitSignal::SafeEvent
                },
                obj = obj_mon.recv() => {
                    cancel_task.cancel();
                   let unwrapped_obj = obj.expect("[FATAL] Objective monitor hung up!");
                    ExecExitSignal::NewObjectiveEvent(unwrapped_obj)
            }
        }
    }

    async fn exec_task(&self, context: Arc<StateContext>, task: Task) -> ExecExitSignal {
        match task.task_type() {
            BaseTask::SwitchState(switch) => self.base.get_task(context, *switch).await,
            _ => {
                panic!(
                    "[FATAL] Illegal task type {} for state {}!",
                    task.task_type(),
                    Self::MODE_NAME
                )
            }
        }
        ExecExitSignal::Continue
    }

    async fn safe_handler(&self, context: Arc<StateContext>) -> OpExitSignal {
        FlightComputer::escape_safe(context.k().f_cont()).await;
        context.o_ch_lock().write().await.finish(
            context
                .o_ch_clone()
                .await
                .i_entry()
                .new_from_pos(context.k().f_cont().read().await.current_pos()),
            self.safe_mode_rationale(),
        );
        OpExitSignal::ReInit(Box::new(self.clone()))
    }

    async fn objective_handler(
        &self,
        context: Arc<StateContext>,
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
                let base = self.base.handle_b_o(context, b_obj).await;
                Some(OpExitSignal::ReInit(Box::new(Self {base})))
            }
            ObjectiveType::KnownImage { zone, optic_required, coverage_required } => {
                let k_obj = KnownImgObjective::new(
                    obj.id(),
                    String::from(obj.name()),
                    obj.start(),
                    obj.end(),
                    *zone,
                    *optic_required,
                    *coverage_required
                );
                // TODO: return ZonedObjectivePrepMode
                todo!();
                //OpExitSignal::ReInit(Box::New())
            }
        }
    }

    async fn exit_mode(&self, _: Arc<StateContext>) -> Box<dyn GlobalMode> {
        Box::new(Self {
            base: self.base.exit_base(),
        })
    }
}
