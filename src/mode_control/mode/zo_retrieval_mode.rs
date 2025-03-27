use crate::flight_control::camera_controller::CameraController;
use crate::flight_control::{
    common::vec2d::Vec2D,
    flight_computer::FlightComputer,
    flight_state::{FlightState, TRANS_DEL},
    objective::known_img_objective::KnownImgObjective,
    task::base_task::{BaseTask, Task},
};
use crate::mode_control::{
    mode::{global_mode::GlobalMode, orbit_return_mode::OrbitReturnMode},
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, OptOpExitSignal, WaitExitSignal},
};
use crate::{DT_0_STD, error, fatal, log, warn};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;
use std::{pin::Pin, sync::Arc};
use tokio::sync::Mutex;
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub struct ZORetrievalMode {
    target: KnownImgObjective,
    add_target: Option<Vec2D<I32F32>>,
    unwrapped_pos: Arc<Mutex<Vec2D<I32F32>>>,
}

impl ZORetrievalMode {
    const MODE_NAME: &'static str = "ZORetrievalMode";
    const SINGLE_TARGET_ACQ_DT: TimeDelta = TimeDelta::seconds(10);
    pub fn new(
        target: KnownImgObjective,
        add_target: Option<Vec2D<I32F32>>,
        unwrapped_pos: Vec2D<I32F32>,
    ) -> Self {
        let unwrapped_lock = Arc::new(Mutex::new(unwrapped_pos));
        Self { target, add_target, unwrapped_pos: unwrapped_lock }
    }

    async fn get_img_fut(
        second_target: Option<Vec2D<I32F32>>,
        unwrapped_pos: Vec2D<I32F32>,
        context: &Arc<ModeContext>,
    ) -> (
        DateTime<Utc>,
        Pin<Box<dyn Future<Output = ()> + Send + Sync>>,
    ) {
        if let Some(add_target) = second_target {
            let current_vel = context.k().f_cont().read().await.current_vel();
            let to_target = {
                let wrapped = unwrapped_pos.wrap_around_map();
                wrapped.unwrapped_to(&add_target)
            };
            let target_traversal_dt =
                TimeDelta::seconds((to_target.abs() / current_vel.abs()).to_num::<i64>());
            let t_end = Utc::now() + Self::SINGLE_TARGET_ACQ_DT * 2 + target_traversal_dt;
            let fut = FlightComputer::turn_for_2nd_target(context.k().f_cont(), add_target, t_end);
            (t_end, Box::pin(fut))
        } else {
            let sleep_dur_std = Self::SINGLE_TARGET_ACQ_DT.to_std().unwrap_or(DT_0_STD);
            (
                Utc::now() + Self::SINGLE_TARGET_ACQ_DT,
                Box::pin(tokio::time::sleep(sleep_dur_std)),
            )
        }
    }

    async fn exec_img_task(target: KnownImgObjective,  unwrapped_target: Vec2D<I32F32>, second_target: Option<Vec2D<I32F32>>, context: Arc<ModeContext>, c_tok: CancellationToken) {
        let offset = Vec2D::new(target.zone()[0], target.zone()[1]).to_unsigned();
        let dim = Vec2D::new(target.width(), target.height()).to_unsigned();

        let c_cont = context.k().c_cont();
        let (deadline, add_fut) = Self::get_img_fut(second_target, unwrapped_target, &context).await;
        let f_cont = context.k().f_cont();
        let mut zoned_objective_image_buffer = None;
        let img_fut = c_cont.execute_zo_target_cycle(f_cont, deadline,&mut zoned_objective_image_buffer, offset, dim);
        tokio::pin!(add_fut);
        tokio::select! {
            () = img_fut => FlightComputer::stop_ongoing_burn(context.k().f_cont()).await,
            () = &mut add_fut => (),
            () = c_tok.cancelled() => {
                warn!("Zoned Objective image Task has been cancelled. Cleaning up!");
                FlightComputer::stop_ongoing_burn(context.k().f_cont()).await;
            }
        }
        let c_cont = context.k().c_cont();
        let id = target.id();
        let img_path = Some(CameraController::generate_zo_img_path(id));
        c_cont.export_and_upload_objective_png(id, offset, dim, img_path, zoned_objective_image_buffer.as_ref()).await.unwrap_or_else(
            |e| {
                error!("Error exporting and uploading objective image: {e}");
            },
        );
    }
}

#[async_trait]
impl GlobalMode for ZORetrievalMode {
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let mut unwrapped_pos = self.unwrapped_pos.lock().await;
        let fut = FlightComputer::detumble_to(
            context.k().f_cont(),
            *unwrapped_pos,
            self.target.optic_required(),
        );
        let safe_mon = context.super_v().safe_mon();
        let target_t;
        let wrapped_target;
        let mut handle = tokio::spawn(fut);
        tokio::select! {
            join = &mut handle => {
                let res = join.ok().unwrap();
                wrapped_target =  res.1;
                target_t = res.0;
            },
            () = safe_mon.notified() => {
                handle.abort();
                return self.safe_handler(context).await;
            }
        }
        *unwrapped_pos = wrapped_target;
        drop(unwrapped_pos);
        let t_cont = context.k().t_cont();
        t_cont.clear_schedule().await; // Just to be sure
        t_cont
            .schedule_retrieval_phase(
                target_t,
                wrapped_target.wrap_around_map(),
                self.target.optic_required(),
            )
            .await;
        OpExitSignal::Continue
    }

    async fn exec_task_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
    ) -> WaitExitSignal {
        let safe_mon = context.super_v().safe_mon();
        let dt = (due - Utc::now()).to_std().unwrap_or(DT_0_STD);
        tokio::select! {
            () = FlightComputer::wait_for_duration(dt, false) => {
                WaitExitSignal::Continue
            },
            () = safe_mon.notified() => {
                WaitExitSignal::SafeEvent
            }
        }
    }

    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal {
        match task.task_type() {
            BaseTask::TakeImage(_) => {
                let safe_mon = context.super_v().safe_mon();
                let c_tok = CancellationToken::new();
                let c_tok_clone = c_tok.clone();
                let context_clone = Arc::clone(&context);
                let second_target = self.add_target;
                let unwrapped_target = *self.unwrapped_pos.lock().await;
                let target = self.target.clone();
                let img_handle = tokio::spawn(async move {
                    Self::exec_img_task(target, unwrapped_target, second_target, context_clone, c_tok_clone).await;
                });
                tokio::pin!(img_handle);
                tokio::select! {
                    _ = &mut img_handle => { },
                    () = safe_mon.notified() => {
                        c_tok.cancel();
                        img_handle.await.unwrap_or_else(|e| {
                            error!("Error joining zo image task: {e}");
                        });
                        return ExecExitSignal::SafeEvent;
                    }
                }
            }
            BaseTask::SwitchState(switch) => {
                let f_cont = context.k().f_cont();
                if matches!(
                    switch.target_state(),
                    FlightState::Acquisition | FlightState::Charge
                ) {
                    FlightComputer::set_state_wait(f_cont, switch.target_state()).await;
                } else {
                    fatal!("Illegal target state!");
                }
            }
            BaseTask::ChangeVelocity(_) => {
                error!("Change Velocity task is forbidden in ZORetrievalMode.");
            }
        }
        ExecExitSignal::Continue
    }

    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        FlightComputer::escape_safe(context.k().f_cont(), false).await;
        let (vel, pos) = {
            let f_cont_locked = context.k().f_cont();
            let f_cont = f_cont_locked.read().await;
            (f_cont.current_vel(), f_cont.current_pos())
        };
        let to_target = pos.to(&*self.unwrapped_pos.lock().await);
        let angle = vel.angle_to(&to_target).abs();
        if angle < I32F32::lit("10.0") {
            let time_cond = {
                let state = context.k().f_cont().read().await.state();
                if state == FlightState::Acquisition {
                    to_target.abs() > I32F32::lit("10.0") * angle
                } else {
                    let transition = I32F32::from_num(
                        TRANS_DEL.get(&(state, FlightState::Acquisition)).unwrap().as_secs(),
                    );
                    to_target.abs() > I32F32::lit("10.0") * angle + transition
                }
            };
            if time_cond {
                log!("Objective still reachable after safe event, staying in ZORetrievalMode");
                FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Acquisition)
                    .await;
                return OpExitSignal::ReInit(Box::new(self.clone()));
            }
        }
        warn!("Objective not reachable after safe event, exiting ZORetrievalMode");
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.out_of_orbit_rationale(),
        );
        OpExitSignal::ReInit(Box::new(OrbitReturnMode::new()))
    }

    async fn zo_handler(&self, _: &Arc<ModeContext>, _: KnownImgObjective) -> OptOpExitSignal {
        unimplemented!()
    }

    async fn bo_event_handler(&self, _: &Arc<ModeContext>) -> OptOpExitSignal { unimplemented!() }

    async fn exit_mode(&self, context: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        context.o_ch_lock().write().await.finish(
            context.k().f_cont().read().await.current_pos(),
            self.tasks_done_rationale(),
        );
        Box::new(OrbitReturnMode::new())
    }
}
