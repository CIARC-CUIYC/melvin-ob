use crate::imaging::CameraAngle;
use chrono::{DateTime, Utc};

/// Represents an objective focused on capturing a secret image.
#[derive(Debug, Clone)]
pub struct SecretImgObjective {
    id: usize,
    name: String,
    start: DateTime<Utc>,
    end: DateTime<Utc>,
    optic_required: CameraAngle,
    coverage_required: f32,
}

impl SecretImgObjective {
    /// Creates a new [`SecretImgObjective`].
    ///
    /// # Arguments
    /// - `id`: The unique identifier for the objective.
    /// - `name`: The name of the objective.
    /// - `start`: The start time of the objective, specified as a `DateTime<Utc>`.
    /// - `end`: The end time of the objective, specified as a `DateTime<Utc>`.
    /// - `optic_required`: The required camera angle for capturing the image.
    /// - `coverage_required`: The required coverage area for image processing.
    ///
    /// # Returns
    /// A new [`SecretImgObjective`] instance.
    pub fn new(
        id: usize,
        name: String,
        start: DateTime<Utc>,
        end: DateTime<Utc>,
        optic_required: CameraAngle,
        coverage_required: f32,
    ) -> Self {
        Self { id, name, start, end, optic_required, coverage_required }
    }
}
