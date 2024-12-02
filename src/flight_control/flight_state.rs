use std::collections::HashMap;
use std::sync::LazyLock;
use std::time::Duration;
use strum_macros::Display;

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
    pub fn get_charge_rate(&self) -> f32{
        match self {
            FlightState::Deployment => {-0.025}
            FlightState::Transition => {0.0}
            FlightState::Acquisition => {-0.2}
            FlightState::Charge => {0.2}
            FlightState::Comms => {-0.016}
            FlightState::Safe => {0.05}
        }
    }
}

impl From<&str> for FlightState {
    fn from(value: &str) -> Self {
        match value.to_lowercase().as_str() {
            "deployment" => FlightState::Deployment,
            "transition" => FlightState::Transition,
            "acquisition" => FlightState::Acquisition,
            "charge" => FlightState::Charge,
            "comms" => FlightState::Comms,
            _ => FlightState::Safe, // TODO: conversion error should be logged
        }
    }
}

impl From<FlightState> for &'static str {
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

pub static TRANSITION_DELAY_LOOKUP: LazyLock<HashMap<(FlightState, FlightState), Duration>> =
    LazyLock::new(|| {
        const TRANSITION_TOLERANCE: u64 = 2;
        let mut lookup = HashMap::new();
        let transition_times = vec![
            // Deployment transitions
            (
                FlightState::Deployment,
                FlightState::Acquisition,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Deployment,
                FlightState::Charge,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Deployment,
                FlightState::Comms,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            // Acquisition transitions
            (
                FlightState::Acquisition,
                FlightState::Deployment,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Acquisition,
                FlightState::Charge,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Acquisition,
                FlightState::Comms,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            // Charge transitions
            (
                FlightState::Charge,
                FlightState::Deployment,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Charge,
                FlightState::Acquisition,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Charge,
                FlightState::Comms,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            // Comms transitions
            (
                FlightState::Comms,
                FlightState::Deployment,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Comms,
                FlightState::Acquisition,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Comms,
                FlightState::Charge,
                Duration::from_secs(180+TRANSITION_TOLERANCE),
            ),
            // Safe transitions
            (
                FlightState::Safe,
                FlightState::Deployment,
                Duration::from_secs(1200+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Safe,
                FlightState::Acquisition,
                Duration::from_secs(1200+TRANSITION_TOLERANCE),
            ),
            (
                FlightState::Safe,
                FlightState::Charge,
                Duration::from_secs(1200+TRANSITION_TOLERANCE),
            ),
        ];

        for (from, to, duration) in transition_times {
            lookup.insert((from, to), duration);
        }
        lookup
    });
