use crate::flight_control::objective::known_img_objective::KnownImgObjective;
use crate::mode_control::base_mode::BaseMode;
use crate::mode_control::mode::global_mode::GlobalMode;

pub enum OpExitSignal {
    ReInit(Box<dyn GlobalMode>),
    Continue,
}

pub enum ExecExitSignal {
    Continue,
    SafeEvent,
    NewZOEvent(KnownImgObjective),
}

pub enum WaitExitSignal {
    Continue,
    SafeEvent,
    NewZOEvent(KnownImgObjective),
    BOEvent,
    RescheduleEvent
}

pub enum BaseWaitExitSignal {
    Continue,
    ReSchedule,
}
