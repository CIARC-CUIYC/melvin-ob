use crate::flight_control::flight_state::FlightState;

#[derive(Debug, Clone, Copy)]
pub enum AtomicDecision {
    StayInCharge,
    StayInAcquisition,
    SwitchToCharge,
    SwitchToAcquisition,
}

impl AtomicDecision {
    pub fn stay(state: usize) -> Self {
        if state == 0 {
            AtomicDecision::StayInCharge
        } else if state == 1 {
            AtomicDecision::StayInAcquisition
        } else {
            panic!("[FATAL] Invalid state for stay decision")
        }
    }

    pub fn switch(to_state: usize) -> Self {
        if to_state == 0 {
            AtomicDecision::SwitchToCharge
        } else if to_state == 1 {
            AtomicDecision::SwitchToAcquisition
        } else {
            panic!("[FATAL] Invalid state for stay decision")
        }
    }
}
