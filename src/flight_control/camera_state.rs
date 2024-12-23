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
    pub fn get_square_radius(&self) -> u16 {
        CAMERA_SQUARE_RAD_LOOKUP[self]
    }
    pub fn get_square_unit_length(&self) -> u16 {
        CAMERA_SCALE_LOOKUP[self]
    }
}

impl From<&str> for CameraAngle {
    fn from(value: &str) -> Self {
        match value.to_lowercase().as_str() {
            "narrow" => CameraAngle::Narrow,
            "wide" => CameraAngle::Wide,
            _ => CameraAngle::Normal, // TODO: conversion error should be logged
        }
    }
}

impl From<CameraAngle> for &'static str {
    fn from(value: CameraAngle) -> Self {
        match value {
            CameraAngle::Narrow => "narrow",
            CameraAngle::Normal => "normal",
            CameraAngle::Wide => "wide",
        }
    }
}

static CAMERA_SQUARE_RAD_LOOKUP: LazyLock<HashMap<CameraAngle, u16>> = LazyLock::new(|| {
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

static CAMERA_SCALE_LOOKUP: LazyLock<HashMap<CameraAngle, u16>> = LazyLock::new(|| {
    let mut lookup = HashMap::new();
    let transition_widths = vec![
        (CameraAngle::Narrow, 600),
        (CameraAngle::Normal, 800),
        (CameraAngle::Wide, 1000),
    ];

    for (angle, square_unit_length) in transition_widths {
        lookup.insert(angle, square_unit_length);
    }
    lookup
});
