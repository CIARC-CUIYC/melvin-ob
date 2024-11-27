use std::collections::HashMap;
use std::sync::LazyLock;
use std::time::Duration;

#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum FlightState {
    Deployment,
    Transition,
    Acquisition,
    Charge,
    Comms,
    Safe,
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

impl From<FlightState> for &'static str{
    fn from(value: FlightState) -> Self {
        match value{
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
        let mut lookup = HashMap::new();
        let transition_times = vec![
            // Deployment transitions
            (FlightState::Deployment, FlightState::Acquisition, Duration::from_secs(180)),
            (FlightState::Deployment, FlightState::Charge, Duration::from_secs(180)),
            (FlightState::Deployment, FlightState::Comms, Duration::from_secs(180)),
            // Acquisition transitions
            (FlightState::Acquisition, FlightState::Deployment, Duration::from_secs(180)),
            (FlightState::Acquisition, FlightState::Charge, Duration::from_secs(180)),
            (FlightState::Acquisition, FlightState::Comms, Duration::from_secs(180)),
            // Charge transitions
            (FlightState::Charge, FlightState::Deployment, Duration::from_secs(180)),
            (FlightState::Charge, FlightState::Acquisition, Duration::from_secs(180)),
            (FlightState::Charge, FlightState::Comms, Duration::from_secs(180)),
            // Comms transitions
            (FlightState::Comms, FlightState::Deployment, Duration::from_secs(180)),
            (FlightState::Comms, FlightState::Acquisition, Duration::from_secs(180)),
            (FlightState::Comms, FlightState::Charge, Duration::from_secs(180)),
            // Safe transitions
            (FlightState::Safe, FlightState::Deployment, Duration::from_secs(1200)),
            (FlightState::Safe, FlightState::Acquisition, Duration::from_secs(1200)),
            (FlightState::Safe, FlightState::Charge, Duration::from_secs(1200)),
        ];

        for (from, to, duration) in transition_times {
            lookup.insert((from, to), duration);
        }
        lookup
    });

