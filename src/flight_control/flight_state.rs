use crate::{DT_0, fatal};
use chrono::TimeDelta;
use fixed::types::I32F32;
use num::Zero;
use std::{collections::HashMap, sync::LazyLock, time::Duration};
use strum_macros::Display;

/// Represents the various states of MELVINs flight system.
///
/// Each state corresponds to a specific operational phase of the flight
/// system and has unique characteristics, such as distinct charge rates and
/// transition times to and from other states.
///
/// # Variants
/// - `Deployment`: The initial deployment phase of the flight system.
/// - `Transition`: A transitional state between two operational modes.
/// - `Acquisition`: State where the system is actively acquiring images and changing orbit.
/// - `Charge`: State where the system is primarily charging its batteries.
/// - `Comms`: State where the system is communicating through the high-gain antenna to receive beacon pings.
/// - `Safe`: A safe mode, typically activated in the event of an anomaly or low power.
#[derive(Debug, Display, PartialEq, Eq, Clone, Copy, Hash)]
pub enum FlightState {
    Charge = 0,
    Acquisition = 1,
    Deployment,
    Transition,
    Comms,
    Safe,
}

impl FlightState {
    /// Additional discharge per second when accelerating in acquisition mode
    pub const ACQ_ACC_ADDITION: I32F32 = I32F32::lit("-0.05");

    /// Returns the charge rate for the given flight state.
    ///
    /// Each state has an associated charge rate that indicates the system's power consumption
    /// or charging rate in that state. A negative value implies power consumption,
    /// while a positive value indicates charging.
    ///
    /// # Returns
    /// A `I32F32` value representing the charge rate for the flight state.
    pub fn get_charge_rate(self) -> I32F32 {
        match self {
            FlightState::Deployment => I32F32::lit("-0.025"),
            FlightState::Transition => I32F32::zero(),
            FlightState::Acquisition => I32F32::lit("-0.1"),
            FlightState::Charge => I32F32::lit("0.1"),
            FlightState::Comms => I32F32::lit("-0.008"),
            FlightState::Safe => I32F32::lit("0.025"),
        }
    }

    /// Maps a usize from the dynamic scheduling program to a [`FlightState`].
    pub fn from_dp_usize(i: usize) -> Self {
        match i {
            1 => FlightState::Acquisition,
            0 => FlightState::Charge,
            _ => panic!("Invalid state"),
        }
    }

    /// Maps a [`FlightState`] to a usize from the dynamic scheduling program.
    pub fn to_dp_usize(self) -> usize {
        match self {
            FlightState::Charge => 0,
            FlightState::Acquisition => 1,
            FlightState::Comms => 2,
            _ => panic!("Invalid state"),
        }
    }

    /// Returns the transition time to another mode
    pub fn dt_to(self, other: Self) -> Duration {
        *TRANS_DEL
            .get(&(self, other))
            .unwrap_or_else(|| fatal!("({self}, {other}) not in TRANSITION_DELAY_LOOKUP"))
    }

    pub fn td_dt_to(self, other: Self) -> TimeDelta {
        TimeDelta::from_std(*TRANS_DEL.get(&(self, other)).unwrap_or_else(|| {
            fatal!("({self}, {other}) not in TRANSITION_DELAY_LOOKUP")
        }))
        .unwrap_or(DT_0)
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
            "communication" => FlightState::Comms,
            "safe" => FlightState::Safe,
            _ => panic!("Couldn't convert flight_state string"),
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
            FlightState::Comms => "communication",
            FlightState::Safe => "safe",
        }
    }
}

/// A static lookup table defining the delays needed
/// for transitioning between different flight states.
///
/// The delay for each transition is represented as a `Duration`.
static TRANS_DEL: LazyLock<HashMap<(FlightState, FlightState), Duration>> = LazyLock::new(|| {
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
