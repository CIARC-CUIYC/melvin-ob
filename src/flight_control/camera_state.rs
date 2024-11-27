use std::collections::HashMap;
use std::sync::LazyLock;
use strum_macros::EnumIter;

#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash, EnumIter)]
pub enum CameraAngle {
    Narrow,
    Normal,
    Wide,
}

impl CameraAngle {
    pub fn get_square_radius(&self) -> u16 { CAMERA_SQUARE_RAD_LOOKUP[self] }
}

impl From<&str> for CameraAngle {
    fn from(value: &str) -> Self {
        match value.to_lowercase().as_str() {
            "narrow" => CameraAngle::Narrow,
            "normal" => CameraAngle::Normal,
            "wide" => CameraAngle::Wide,
            _ => CameraAngle::Normal // TODO: conversion error should be logged
        }
    }
}

impl Into<String> for CameraAngle {
    fn into(self) -> String {
        match self {
            CameraAngle::Narrow => String::from("narrow"),
            CameraAngle::Normal => String::from("normal"),
            CameraAngle::Wide => String::from("wide")
        }
    }
}

static CAMERA_SQUARE_RAD_LOOKUP: LazyLock<HashMap<CameraAngle, u16>> =
    LazyLock::new(|| {
        let mut lookup = HashMap::new();
        let transition_times = vec![
            (CameraAngle::Narrow, 300),
            (CameraAngle::Normal, 400),
            (CameraAngle::Wide, 500),
        ];

        for (angle, rad) in transition_times {
            lookup.insert(angle, rad);
        }
        lookup
    });