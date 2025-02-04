/// Represents the different atomic decisions that can be made regarding states.
#[derive(Debug, Clone, Copy)]
pub enum AtomicDecision {
    /// Decision to stay in the charge state.
    StayInCharge,
    /// Decision to stay in the acquisition state.
    StayInAcquisition,
    /// Decision to switch to the charge state.
    SwitchToCharge,
    /// Decision to switch to the acquisition state.
    SwitchToAcquisition,
}

impl AtomicDecision {
    /// Creates a decision to stay in the current state based on the provided state value.
    ///
    /// # Arguments
    /// - `state`: The current state as a `usize`.
    ///   - `0` indicates the charge state.
    ///   - `1` indicates the acquisition state.
    ///
    /// # Returns
    /// - An `AtomicDecision` variant corresponding to staying in the current state.
    ///
    /// # Panics
    /// - If `state` is not `0` or `1`.
    pub fn stay(state: usize) -> Self {
        if state == 0 {
            AtomicDecision::StayInCharge
        } else if state == 1 {
            AtomicDecision::StayInAcquisition
        } else {
            panic!("[FATAL] Invalid state for stay decision")
        }
    }

    /// Creates a decision to switch to a new state based on the provided state value.
    ///
    /// # Arguments
    /// - `to_state`: The target state as a `usize`.
    ///   - `0` indicates the charge state.
    ///   - `1` indicates the acquisition state.
    ///
    /// # Returns
    /// - An `AtomicDecision` variant corresponding to switching to the target state.
    ///
    /// # Panics
    /// - If `to_state` is not `0` or `1`.
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
