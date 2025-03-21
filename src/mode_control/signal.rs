use crate::flight_control::objective::objective_base::ObjectiveBase;
use crate::mode_control::{base_mode::BaseWaitExitSignal, mode::global_mode::GlobalMode};

pub enum OpExitSignal {
    ReInit(Box<dyn GlobalMode>),
    Continue,
}

pub enum ExecExitSignal {
    Continue,
    SafeEvent,
    NewObjectiveEvent(ObjectiveBase),
}

pub enum WaitExitSignal {
    Continue,
    SafeEvent,
    NewObjectiveEvent(ObjectiveBase),
    BODoneEvent(BaseWaitExitSignal),
}
