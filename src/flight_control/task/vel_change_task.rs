use crate::flight_control::orbit::BurnSequence;

#[derive(Debug, Copy, Clone)]
pub enum VelocityChangeTaskRationale {
    Correctional,
    OrbitEscape,
    OrbitEnter,
}

/// Represents a task for executing a velocity change, using a burn sequence.
#[derive(Debug)]
pub struct VelocityChangeTask {
    /// The burn sequence defining the velocity change.
    burn: BurnSequence,
    /// The reason why this velocity change is performed
    rationale: VelocityChangeTaskRationale,
}

impl VelocityChangeTask {
    /// Creates a new [`VelocityChangeTask`] with the specified burn sequence.
    ///
    /// # Arguments
    /// - `burn`: The burn sequence to be executed as part of this task.
    ///
    /// # Returns
    /// - A new instance of [`VelocityChangeTask`].
    pub fn new(burn: BurnSequence, rationale: VelocityChangeTaskRationale) -> Self {
        Self { burn, rationale }
    }

    /// Retrieves a reference to the burn sequence associated with the task.
    ///
    /// # Returns
    /// - An immutable reference to the [`BurnSequence`].
    pub fn burn(&self) -> &BurnSequence { &self.burn }

    /// Retrieves the reason for the velocity change
    ///
    /// # Returns
    /// - A [`VelocityChangeTaskRationale`]
    pub fn rationale(&self) -> VelocityChangeTaskRationale { self.rationale }
}
