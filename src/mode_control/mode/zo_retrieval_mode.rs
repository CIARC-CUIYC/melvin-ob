use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::imaging::map_image::OffsetZOImage;
use crate::flight_control::task::TaskController;
use crate::flight_control::task::base_task::BaseTask;
use crate::flight_control::{
    objective::{known_img_objective::KnownImgObjective, objective_base::ObjectiveBase},
    task::base_task::Task,
};
use crate::mode_control::{
    base_mode::BaseWaitExitSignal,
    mode::global_mode::GlobalMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal},
};
use crate::{DT_0_STD, error, info};
use async_trait::async_trait;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;
use std::sync::Arc;
use std::time::Duration;
use crate::mode_control::mode::orbit_return_mode::OrbitReturnMode;

#[derive(Clone)]
pub struct ZORetrievalMode {
    target: KnownImgObjective,
    targets: Vec<Vec2D<I32F32>>,
}

impl ZORetrievalMode {
    const MODE_NAME: &'static str = "ZORetrievalMode";
    const SINGLE_TARGET_ACQ_DT: TimeDelta = TimeDelta::seconds(10);
    pub fn new(target: KnownImgObjective, targets: Vec<Vec2D<I32F32>>) -> Self {
        Self { target, targets }
    }
}

#[async_trait]
impl GlobalMode for ZORetrievalMode {

    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let target_t = FlightComputer::detumble_to(
            context.k().f_cont(),
            self.targets[0],
            self.target.optic_required(),
        )
        .await;
        context
            .k()
            .t_cont()
            .schedule_zo_image(target_t, self.targets[0], self.target.optic_required())
            .await;
        OpExitSignal::Continue
    }

    async fn exec_task_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
    ) -> WaitExitSignal {
        let safe_mon = context.super_v().safe_mon();
        let mut obj_mon = context.obj_mon().write().await;
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
                WaitExitSignal::NewObjectiveEvent(obj)
            }
        }
    }

    async fn exec_task(&self, context: Arc<ModeContext>, task: Task) -> ExecExitSignal {
        match task.task_type() {
            BaseTask::TakeImage(task) => {
                let bottom_left = Vec2D::new(
                    I32F32::from_num(self.target.zone()[0]),
                    I32F32::from_num(self.target.zone()[1]),
                );
                let dimensions =
                    Vec2D::new(self.target.width(), self.target.height()).to_unsigned();
                let mut buffer = OffsetZOImage::new(bottom_left, dimensions);
                let deadline = Utc::now() + Self::SINGLE_TARGET_ACQ_DT;
                let c_cont = context.k().c_cont();
                c_cont.execute_zo_single_target_cycle(context.k().f_cont(), &mut buffer, deadline).await;
            }
            _ => error!(
                "Invalid task type in ZORetrievalMode: {:?}",
                task.task_type()
            ),
        }
        ExecExitSignal::Continue
    }

    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        todo!();
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
        todo!()
    }

    async fn b_o_done_handler(&self, _: Arc<ModeContext>, _: BaseWaitExitSignal) -> OpExitSignal {
        unimplemented!()
    }

    async fn exit_mode(&self, _: Arc<ModeContext>) -> Box<dyn GlobalMode> { 
        info!("Exiting ZORetrievalMode");
        Box::new(OrbitReturnMode::new())
    }
}
