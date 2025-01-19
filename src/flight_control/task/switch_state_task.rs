use crate::flight_control::common::pinned_dt::PinnedTimeDelay;
use crate::flight_control::flight_state::FlightState;

#[derive(Debug, Copy, Clone)]
pub struct SwitchStateTask {
    /// The target state to switch to.
    target_state: FlightState,
}

impl SwitchStateTask {
    pub fn new(target_state: FlightState) -> Option<Self> {
        match target_state {
            FlightState::Charge | FlightState::Comms | FlightState::Acquisition => {
                Some(Self { target_state })
            }
            _ => None,
        }
    }
    
    pub fn target_state(&self) -> FlightState { self.target_state }
}
