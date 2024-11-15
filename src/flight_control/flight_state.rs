use crate::flight_control::camera_state::CameraAngle;

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
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
            "safe" => FlightState::Safe,
            _ => FlightState::Safe, // TODO: conversion error should be logged
        }
    }
}

impl Into<String> for FlightState {
    fn into<'a>(self) -> String {
        match self{
            FlightState::Deployment => String::from("deployment"),
            FlightState::Transition => String::from("transition"),
            FlightState::Acquisition => String::from("acquisition"),
            FlightState::Charge => String::from("charge"),
            FlightState::Comms => String::from("comms"),
            FlightState::Safe => String::from("safe"),
        }
    }
}
