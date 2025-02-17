use crate::flight_control::common::vec2d::Vec2D;
use fixed::types::I32F32;

/// Represents a position in an orbit with associated metadata.
#[derive(Debug, Copy, Clone)]
pub struct IndexedOrbitPosition {
    /// The timestamp representing the current time of this position.
    t: chrono::DateTime<chrono::Utc>,
    /// The index representing the current index in the orbits `done`-box.
    index: usize,
    /// The 2D positional vector of this point in the orbit.
    pos: Vec2D<I32F32>,
    /// The period of the orbit.
    period: usize,
}

impl IndexedOrbitPosition {
    /// Creates a new `IndexedOrbitPosition` instance.
    ///
    /// # Parameters
    /// - `index`: The index in the orbit.
    /// - `period`: The period of the orbit.
    /// - `pos`: The 2D position vector for this point in the orbit.
    ///
    /// # Returns
    /// A new `IndexedOrbitPosition` instance with the current UTC time.
    pub fn new(index: usize, period: usize, pos: Vec2D<I32F32>) -> Self {
        Self {
            t: chrono::Utc::now(),
            index,
            pos,
            period,
        }
    }

    /// Returns the timestamp of the position.
    pub fn t(&self) -> chrono::DateTime<chrono::Utc> {
        self.t
    }

    /// Returns the 2D positional vector of the orbit.
    pub fn pos(&self) -> Vec2D<I32F32> {
        self.pos
    }

    /// Returns the current index in the orbit.
    pub fn index(&self) -> usize {
        self.index
    }

    /// Returns the period of the orbit.
    pub fn period(&self) -> usize {
        self.period
    }

    /// Calculates the ranges from the current index to now, optionally applying a shift.
    ///
    /// # Parameters
    /// - `shift`: An optional value to adjust the calculation of the current index.
    ///
    /// # Returns
    /// A vector of tuples representing the ranges as `(start, end)` for the orbit indices.
    pub fn get_ranges_to_now(&self, shift: Option<usize>) -> Vec<(usize, usize)> {
        let end = {
            if let Some(sh) = shift {
                (((self.index_now() - sh) % self.period) + self.period) % self.period
            } else {
                self.index_now() % self.period
            }
        };
        if end < self.index {
            vec![(self.index, self.period), (0, end)]
        } else {
            vec![(self.index, self.index_now() % self.period)]
        }
    }

    /// Maps ranges of indices, ensuring they wrap around correctly for a maximum range.
    ///
    /// # Parameters
    /// - `ranges`: A vector of tuples representing input ranges as `(start, end)`.
    /// - `max`: The maximum range for the indices.
    ///
    /// # Returns
    /// A vector of tuples representing mapped ranges as `(start, end)` for the orbit indices.
    #[allow(clippy::cast_sign_loss)]
    pub fn map_ranges(ranges: &Vec<(isize, isize)>, max: isize) -> Vec<(usize, usize)> {
        let mut mapped_ranges = Vec::new();
        for range in ranges {
            let start = (((range.0 % max) + max) % max) as usize;
            let end = (((range.1 % max) + max) % max) as usize;
            if start < end {
                mapped_ranges.push((start, end));
            } else {
                mapped_ranges.push((start, (max - 1) as usize));
                mapped_ranges.push((0, end));
            }
        }
        mapped_ranges
    }

    /// Creates a new `IndexedOrbitPosition` with the given 2D position vector.
    ///
    /// # Parameters
    /// - `pos`: The new 2D position vector.
    ///
    /// # Returns
    /// A new `IndexedOrbitPosition` instance with updated position and time.
    pub fn new_from_pos(&self, pos: Vec2D<I32F32>) -> Self {
        Self {
            t: chrono::Utc::now(),
            index: self.index_now(),
            pos,
            period: self.period,
        }
    }

    /// Creates a new `IndexedOrbitPosition` with a future timestamp and given 2D position vector.
    ///
    /// # Parameters
    /// - `pos`: The new 2D position vector.
    /// - `dt`: The time difference to calculate the future timestamp.
    ///
    /// # Returns
    /// A new `IndexedOrbitPosition` instance with updated position and future timestamp.
    pub fn new_from_future_pos(&self, pos: Vec2D<I32F32>, dt: chrono::TimeDelta) -> Self {
        Self {
            t: self.t + dt,
            index: self.index_then(dt),
            pos,
            period: self.period,
        }
    }

    /// Calculates the current index in the orbit based on the elapsed time.
    ///
    /// # Returns
    /// The current index in the orbit.
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    fn index_now(&self) -> usize {
        (self.index + (chrono::Utc::now() - self.t).num_seconds() as usize) % self.period
    }

    /// Calculates the index in the orbit for a given time offset.
    ///
    /// # Parameters
    /// - `dt`: The time difference to calculate the future index.
    ///
    /// # Returns
    /// The future index in the orbit.
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    fn index_then(&self, dt: chrono::TimeDelta) -> usize {
        (self.index + dt.num_seconds() as usize) % self.period
    }
}
