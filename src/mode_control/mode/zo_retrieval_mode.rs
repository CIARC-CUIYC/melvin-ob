use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::flight_state::{FlightState, TRANS_DEL};
use crate::flight_control::imaging::map_image::OffsetZOImage;
use crate::flight_control::task::base_task::BaseTask;
use crate::flight_control::{
    objective::known_img_objective::KnownImgObjective, task::base_task::Task,
};
use crate::mode_control::mode::orbit_return_mode::OrbitReturnMode;
use crate::mode_control::{
    mode::global_mode::GlobalMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use crate::{DT_0_STD, error, info, log, warn};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;
use std::sync::Arc;

#[derive(Clone)]
pub struct ZORetrievalMode {
    target: KnownImgObjective,
    unwrapped_pos: Vec2D<I32F32>,
}

impl ZORetrievalMode {
    const MODE_NAME: &'static str = "ZORetrievalMode";
    const SINGLE_TARGET_ACQ_DT: TimeDelta = TimeDelta::seconds(10);
    pub fn new(target: KnownImgObjective, unwrapped_pos: Vec2D<I32F32>) -> Self {
        Self {
            target,
            unwrapped_pos,
        }
    }
}

#[async_trait]
impl GlobalMode for ZORetrievalMode {
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let target_t = FlightComputer::detumble_to(
            context.k().f_cont(),
            self.unwrapped_pos,
            self.target.optic_required(),
        )
        .await;
        let t_cont = context.k().t_cont();
        t_cont.clear_schedule().await; // Just to be sure
        t_cont
            .schedule_retrieval_phase(
                target_t,
                self.unwrapped_pos.wrap_around_map(),
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
        let mut obj_mon = context.zo_mon().write().await;
        let dt = (due - Utc::now()).to_std().unwrap_or(DT_0_STD);
        tokio::select! {
            () = tokio::time::sleep(dt) => {
                WaitExitSignal::Continue
            },
            () = safe_mon.notified() => {
                WaitExitSignal::SafeEvent
            },
            msg =  obj_mon.recv() => {
                let obj = msg.expect("[FATAL] Objective monitor hung up!");
                WaitExitSignal::NewZOEvent(obj)
            }
        }
    }

    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal {
        match task.task_type() {
            BaseTask::TakeImage(_) => {
                let bottom_left = Vec2D::new(
                    I32F32::from_num(self.target.zone()[0]),
                    I32F32::from_num(self.target.zone()[1]),
                );
                let dimensions =
                    Vec2D::new(self.target.width(), self.target.height()).to_unsigned();
                let mut buffer = OffsetZOImage::new(bottom_left, dimensions);
                let deadline = Utc::now() + Self::SINGLE_TARGET_ACQ_DT;
                let c_cont = context.k().c_cont();
                c_cont
                    .execute_zo_single_target_cycle(context.k().f_cont(), &mut buffer, deadline)
                    .await;
            }
            _ => error!(
                "Invalid task type in ZORetrievalMode: {:?}",
                task.task_type()
            ),
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
        let to_target = pos.to(&self.unwrapped_pos);
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
        OpExitSignal::ReInit(Box::new(OrbitReturnMode::new()))
    }

    async fn zo_handler(
        &self,
        context: Arc<ModeContext>,
        task: KnownImgObjective,
    ) -> Option<OpExitSignal> {
        context.k_buffer().lock().await.push(task);
        None
    }

    fn bo_event_handler(&self) -> Option<OpExitSignal> { unimplemented!() }

    async fn exit_mode(&self, _: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        info!("Exiting ZORetrievalMode");
        Box::new(OrbitReturnMode::new())
    }
}
