use std::collections::HashMap;
use std::sync::LazyLock;
use std::time::Duration;
use strum_macros::Display;

/// Represents the various states of a flight system.
///
/// Each state corresponds to a specific operational phase of the flight
/// system and has unique characteristics, such as distinct charge rates and
/// transition times to and from other states.
///
/// # Variants
/// - `Deployment`: The initial deployment phase of the flight system.
/// - `Transition`: A transitional state between two operational modes.
/// - `Acquisition`: State where the system is actively acquiring data and changing orbit.
/// - `Charge`: State where the system is primarily charging its batteries.
/// - `Comms`: State where the system is communicating through the high-gain antenna.
/// - `Safe`: A safe mode, typically activated in the event of an anomaly or low power.
#[derive(Debug, Display, PartialEq, Eq, Clone, Copy, Hash)]
pub enum FlightState {
    Deployment,
    Transition,
    Acquisition,
    Charge,
    Comms,
    Safe,
}

impl FlightState {
    /// Returns the charge rate for the given flight state.
    ///
    /// Each state has an associated charge rate that indicates the system's power consumption
    /// or charging rate in that state. A negative value implies power consumption,
    /// while a positive value indicates charging.
    ///
    /// # Returns
    /// A `f32` value representing the charge rate for the flight state.
    pub fn get_charge_rate(self) -> f32{
        match self {
            FlightState::Deployment => {-0.025}
            FlightState::Transition => {0.0}
            FlightState::Acquisition => {-0.1}
            FlightState::Charge => {0.1}
            FlightState::Comms => {-0.016}
            FlightState::Safe => {0.05}
        }
    }
}

impl From<&str> for FlightState {
    /// Converts a string value into a `FlightState` enum.
    ///
    /// # Arguments
    /// - `value`: A string slice representing the flight state (`"deployment"`, `"transition"`, 
    ///   `"acquisition"`, `"charge"`, `"comms"` or `"safe"`).
    ///
    /// # Returns
    /// A `FlightState` converted from the input string. 
    /// If the input is an unknown string this defaults to `safe` and logs the error.
    fn from(value: &str) -> Self {
        match value.to_lowercase().as_str() {
            "deployment" => FlightState::Deployment,
            "transition" => FlightState::Transition,
            "acquisition" => FlightState::Acquisition,
            "charge" => FlightState::Charge,
            "comms" => FlightState::Comms,
            "safe" => FlightState::Safe,
            _ => panic!("Couldn't convert flight_state string")// TODO: conversion error should be logged
        }
    }
}

impl From<FlightState> for &'static str {
    /// Converts a `FlightState` variant back into its string representation.
    ///
    /// # Arguments
    /// - `value`: A `FlightState` variant to be converted.
    ///
    /// # Returns
    /// A string slice (`"deployment"`, `"transition"`, `"acquisition"`, `"charge"`, `"comms"`, `"safe"`)
    /// corresponding to the given `FlightState`.
    fn from(value: FlightState) -> Self {
        match value {
            FlightState::Deployment => "deployment",
            FlightState::Transition => "transition",
            FlightState::Acquisition => "acquisition",
            FlightState::Charge => "charge",
            FlightState::Comms => "comms",
            FlightState::Safe => "safe",
        }
    }
}

/// A pre-computed lookup table defining the delays needed
/// for transitioning between different flight states.
///
/// The delay for each transition is represented as a `Duration`.
pub static TRANSITION_DELAY_LOOKUP: LazyLock<HashMap<(FlightState, FlightState), Duration>> =
    LazyLock::new(|| {
        let mut lookup = HashMap::new();
        let transition_times = vec![
            // Deployment transitions
            (
                FlightState::Deployment,
                FlightState::Acquisition,
                Duration::from_secs(180),
            ),
            (
                FlightState::Deployment,
                FlightState::Charge,
                Duration::from_secs(180),
            ),
            (
                FlightState::Deployment,
                FlightState::Comms,
                Duration::from_secs(180),
            ),
            
            // Acquisition transitions
            (
                FlightState::Acquisition,
                FlightState::Deployment,
                Duration::from_secs(180),
            ),
            (
                FlightState::Acquisition,
                FlightState::Charge,
                Duration::from_secs(180),
            ),
            (
                FlightState::Acquisition,
                FlightState::Comms,
                Duration::from_secs(180),
            ),
            
            // Charge transitions
            (
                FlightState::Charge,
                FlightState::Deployment,
                Duration::from_secs(180),
            ),
            (
                FlightState::Charge,
                FlightState::Acquisition,
                Duration::from_secs(180),
            ),
            (
                FlightState::Charge,
                FlightState::Comms,
                Duration::from_secs(180),
            ),
            
            // Comms transitions
            (
                FlightState::Comms,
                FlightState::Deployment,
                Duration::from_secs(180),
            ),
            (
                FlightState::Comms,
                FlightState::Acquisition,
                Duration::from_secs(180),
            ),
            (
                FlightState::Comms,
                FlightState::Charge,
                Duration::from_secs(180),
            ),
            
            // Safe transitions
            (
                FlightState::Safe,
                FlightState::Deployment,
                Duration::from_secs(1200),
            ),
            (
                FlightState::Safe,
                FlightState::Acquisition,
                Duration::from_secs(1200),
            ),
            (
                FlightState::Safe,
                FlightState::Charge,
                Duration::from_secs(1200),
            ),
        ];

        for (from, to, duration) in transition_times {
            lookup.insert((from, to), duration);
        }
        lookup
    });
