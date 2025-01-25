use chrono::{DateTime, TimeDelta, Utc};

/// A structure that represents a delay tied to a specific start time.
/// This is useful for managing time-based operations where a delay is computed
/// relative to a pinned start time (`start_time`).
///
/// # Fields
/// - `start_time`: A `DateTime<Utc>` representing the starting time timestamp.
/// - `delay`: A `TimeDelta` representing the time left for the delay.
///
/// The delay can be adjusted dynamically, and the end time can be calculated
/// based on the current delay.
#[derive(Debug, Clone, Copy)]
pub struct PinnedTimeDelay {
    /// The time when the delay started.
    start_time: DateTime<Utc>,
    /// The amount of time (should be positive) to calculate the end time from the start time.
    delay: TimeDelta,
}

impl PinnedTimeDelay {
    /// Creates a new `PinnedTimeDelay` with the specified delay, starting at the current time (`Utc::now()`).
    ///
    /// # Arguments
    /// * `delta` - The initial `TimeDelta` to represent the delay.
    ///
    /// # Returns
    /// A new instance of `PinnedTimeDelay`.
    pub fn new(delta: TimeDelta) -> Self {
        Self {
            start_time: Utc::now(),
            delay: delta,
        }
    }
    
    pub fn from_end(end: DateTime<Utc>) -> Self {
        Self {
            start_time: Utc::now(),
            delay: end - Utc::now(),
        }
    }

    /// Subtracts the given `TimeDelta` from the current delay.
    ///
    /// # Arguments
    /// * `delta` - The `TimeDelta` to subtract from the current delay.
    pub fn sub_delay(&mut self, delta: TimeDelta) {
        self.delay -= delta;
    }

    /// Adds the given `TimeDelta` to the current delay.
    ///
    /// # Arguments
    /// * `delta` - The `TimeDelta` to add to the current delay.
    pub fn add_delay(&mut self, delta: TimeDelta) {
        self.delay += delta;
    }

    /// Sets the delay to the specified `TimeDelta`.
    /// This completely overrides the current delay with a new one.
    ///
    /// # Arguments
    /// * `delta` - The new `TimeDelta` to set as the delay.
    pub fn set_delay(&mut self, delta: TimeDelta) {
        self.delay = delta;
    }

    /// Calculates and retrieves the end time based on the `start_time` and the current `delay`.
    ///
    /// # Returns
    /// The end time as a `DateTime<Utc>`.
    pub fn get_end(&self) -> DateTime<Utc> {
        self.start_time + self.delay
    }

    /// Retrieves the start time of the delay.
    ///
    /// # Returns
    /// The `start_time` as a `DateTime<Utc>`.
    pub fn get_start(&self) -> DateTime<Utc> {
        self.start_time
    }

    /// Calculates the remaining time until the end of the delay, based on the current time (`Utc::now()`).
    ///
    /// # Returns
    /// The remaining time as a `TimeDelta`.
    /// Positive values indicate time left, while negative values indicate the delay has passed.
    pub fn time_left(&self) -> TimeDelta {
        self.get_end() - Utc::now()
    }
}
