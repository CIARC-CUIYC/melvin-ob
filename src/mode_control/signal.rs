use chrono::{DateTime, Utc};
use tokio::task::JoinHandle;
use crate::flight_control::objective::known_img_objective::KnownImgObjective;
use super::mode::global_mode::GlobalMode;

pub(super) enum TaskEndSignal {
    Timestamp(DateTime<Utc>),
    Join(JoinHandle<()>),
}

#[derive(Debug)]
pub(crate) enum PeriodicImagingEndSignal {
    KillNow,
    KillLastImage,
}

pub(crate) enum OpExitSignal {
    ReInit(Box<dyn GlobalMode>),
    Continue,
}

pub(crate) enum ExecExitSignal {
    Continue,
    SafeEvent,
    NewZOEvent(KnownImgObjective),
}

pub(crate) enum WaitExitSignal {
    Continue,
    SafeEvent,
    NewZOEvent(KnownImgObjective),
    BOEvent,
}

pub(super) type OptOpExitSignal = Option<OpExitSignal>;
