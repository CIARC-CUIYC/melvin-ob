use crate::flight_control::objective::{
    known_img_objective::KnownImgObjective, objective_base::ObjectiveBase,
};
use crate::mode_control::{base_mode::BaseWaitExitSignal, mode::global_mode::GlobalMode};

pub enum OpExitSignal {
    ReInit(Box<dyn GlobalMode>),
    Continue,
}

pub enum ExecExitSignal {
    Continue,
    SafeEvent,
    NewObjectiveEvent(ObjectiveBase),
    ExitedOrbit(KnownImgObjective),
}

pub enum WaitExitSignal {
    Continue,
    SafeEvent,
    NewObjectiveEvent(ObjectiveBase),
    BODoneEvent(BaseWaitExitSignal),
}
