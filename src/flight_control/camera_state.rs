use fixed::types::I32F32;
use std::{collections::HashMap, sync::LazyLock};
use strum_macros::{Display, EnumIter};

/// Represents different camera angles supported by the system.
///
/// # Variants
/// - `Narrow`: Indicates a narrow FOV for the camera.
/// - `Normal`: Indicates a normal FOV for the camera.
/// - `Wide`: Indicates a wide FOV for the camera.
///
/// These angles are associated with a specific square side length
/// for image processing purposes, available in a pre-computed lookup table.
#[derive(Debug, Display, PartialEq, Eq, Clone, Copy, Hash, EnumIter)]
pub enum CameraAngle {
    Narrow,
    Normal,
    Wide,
}

impl CameraAngle {
    /// Returns the square side length (in pixels) associated with the given camera angle.
    /// The value is retrieved from a pre-computed lookup table `CAMERA_SCALE_LOOKUP`.
    ///
    /// # Returns
    /// A `u16` representing the side length of the square for the given camera angle.
    pub fn get_square_side_length(self) -> u16 { CAMERA_SCALE_LOOKUP[&self] }

    pub fn get_max_speed(self) -> I32F32 { CAMERA_MAX_SPEED_LOOKUP[&self] }
}

impl From<&str> for CameraAngle {
    /// Converts a string value into a `CameraAngle` enum.
    ///
    /// # Arguments
    /// - `value`: A string slice representing the camera angle (`"narrow"`, `"normal"` or `"wide"`).
    ///
    /// # Returns
    /// A `CameraAngle` converted from the input string.
    /// If the input is an unknown string this defaults to `normal` and logs the error.
    fn from(value: &str) -> Self {
        match value.to_lowercase().as_str() {
            "narrow" => CameraAngle::Narrow,
            "wide" => CameraAngle::Wide,
            "normal" => CameraAngle::Normal,
            _ => panic!("Couldn't convert camera_angle string"),
        }
    }
}

impl From<CameraAngle> for &'static str {
    /// Converts a `CameraAngle` enum back into a static string slice representation.
    ///
    /// # Arguments
    /// - `value`: A `CameraAngle` variant to be converted.
    ///
    /// # Returns
    /// A string slice representation of the camera angle (`"narrow"`, `"normal"`, `"wide"`).
    fn from(value: CameraAngle) -> Self {
        match value {
            CameraAngle::Narrow => "narrow",
            CameraAngle::Normal => "normal",
            CameraAngle::Wide => "wide",
        }
    }
}

/// A pre-computed lookup table mapping `CameraAngle` variants to their
/// associated square side lengths (in pixels).
///
/// The side length represents the size of an image square corresponding
/// to a specific camera angle.
static CAMERA_SCALE_LOOKUP: LazyLock<HashMap<CameraAngle, u16>> = LazyLock::new(|| {
    let mut lookup = HashMap::new();
    let transition_widths = vec![
        (CameraAngle::Narrow, 600),
        (CameraAngle::Normal, 800),
        (CameraAngle::Wide, 1000),
    ];

    for (angle, square_side_length) in transition_widths {
        lookup.insert(angle, square_side_length);
    }
    lookup
});

static CAMERA_MAX_SPEED_LOOKUP: LazyLock<HashMap<CameraAngle, I32F32>> = LazyLock::new(|| {
    let mut lookup = HashMap::new();
    let transition_widths = vec![
        (CameraAngle::Narrow, I32F32::lit("10.0")),
        (CameraAngle::Normal, I32F32::lit("50.0")),
        (CameraAngle::Wide, I32F32::MAX),
    ];

    for (angle, square_side_length) in transition_widths {
        lookup.insert(angle, square_side_length);
    }
    lookup
});
