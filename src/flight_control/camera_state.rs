#[derive(PartialEq, Clone, Copy, Debug)]
pub enum CameraAngle {
    Narrow,
    Normal,
    Wide
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
        match self{
            CameraAngle::Narrow => String::from("narrow"),
            CameraAngle::Normal => String::from("normal"),
            CameraAngle::Wide => String::from("wide")
        }
    }
}