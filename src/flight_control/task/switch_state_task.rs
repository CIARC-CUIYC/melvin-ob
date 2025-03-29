use crate::flight_control::flight_state::FlightState;

/// Represents a task to switch to a target flight state.
///
/// This task specifies the desired state the flight system should transition to.
/// The target state must be a valid operational mode.
#[derive(Debug, Copy, Clone)]
pub struct SwitchStateTask {
    /// The target state to switch to.
    target_state: FlightState,
}

impl SwitchStateTask {
    /// Creates a new [`SwitchStateTask`] for a given target state.
    ///
    /// # Arguments
    /// - `target_state`: The desired flight state to switch to.
    ///
    /// # Returns
    /// - `Some(SwitchStateTask)`: If the target state is a valid state for switching.
    /// - `None`: If the target state is not valid for switching.
    ///
    /// # Valid States
    /// - [`FlightState::Charge`]: The state where the system is charging its batteries.
    /// - [`FlightState::Comms`]: The state where the system is communicating via high-gain antenna.
    /// - [`FlightState::Acquisition`]: The state where the system is actively acquiring images.
    pub fn new(target_state: FlightState) -> Option<Self> {
        match target_state {
            FlightState::Charge | FlightState::Comms | FlightState::Acquisition => {
                Some(Self { target_state })
            }
            _ => None,
        }
    }

    /// Returns the target state of the `SwitchStateTask`.
    ///
    /// # Returns
    /// - The `FlightState` this task is targeting.
    pub fn target_state(self) -> FlightState { self.target_state }
}
