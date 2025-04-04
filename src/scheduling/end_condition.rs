use crate::flight_control::{FlightState, orbit::BurnSequence};
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;

/// Represents a scheduling boundary condition for dynamic orbit planning and task execution.
///
/// `EndCondition` defines a specific point in time where the spacecraft must be in a
/// given `FlightState` with at least a specified charge level. This is primarily used
/// when planning tasks (e.g. burns, communication, imaging) to ensure mission-critical
/// resources are met.
///
/// Typically passed to the scheduler to guide the final phase of planning.
pub struct EndCondition {
    /// The desired scheduling terminal charge
    charge: I32F32,
    /// The desired scheduling terminal [`FlightState`]
    state: FlightState,
    /// The desired end of scheduling
    time: DateTime<Utc>,
}

impl EndCondition {
    /// Creates an [`EndCondition`] from a given burn sequence.
    ///
    /// The resulting condition requires being in `Acquisition` mode with
    /// the burn's `min_charge()` at its start time.
    ///
    /// # Arguments
    /// - `burn`: A reference to the [`BurnSequence`] from which to derive the end condition.
    ///
    /// # Returns
    /// - A new [`EndCondition`] configured for the start of the burn.
    pub fn from_burn(burn: &BurnSequence) -> Self {
        Self {
            time: burn.start_i().t(),
            charge: burn.min_charge(),
            state: FlightState::Acquisition,
        }
    }

    /// Returns the absolute time of the end condition.
    pub fn time(&self) -> DateTime<Utc> { self.time }
    /// Returns the required battery level at the end condition time.
    pub fn charge(&self) -> I32F32 { self.charge }
    /// Returns the expected flight state at the end condition time.
    pub fn state(&self) -> FlightState { self.state }

    /// Computes the absolute `TimeDelta` required to charge from 0 to the required `charge()` level.
    ///
    /// This is based on the charge rate defined for `FlightState::Charge`.
    ///
    /// # Returns
    /// - A `TimeDelta` representing the time needed to reach the required charge level.
    pub fn abs_charge_dt(&self) -> TimeDelta {
        let secs = (self.charge() / FlightState::Charge.get_charge_rate()).round().to_num::<i64>();
        TimeDelta::seconds(secs)
    }
}
