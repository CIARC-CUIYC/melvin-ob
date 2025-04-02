use super::{closed_orbit::ClosedOrbit, index::IndexedOrbitPosition};
use crate::util::Vec2D;
use crate::flight_control::FlightComputer;
use crate::info;
use fixed::types::I32F32;
use tokio::sync::RwLock;

/// Represents the characteristics of an orbital path including imaging frequency,
/// orbital period, and the entry position. This struct provides utilities to initialize
/// and manage orbital parameters over time.
#[derive(Debug, Copy, Clone)]
pub struct OrbitCharacteristics {
    /// The maximum time interval between image captures.
    img_dt: I32F32,
    /// The full period of the orbit in terms of iterations.
    orbit_full_period: usize,
    /// The entry position of the orbit indexed in time and position.
    i_entry: IndexedOrbitPosition,
    /// The already performed number of fsm mode switches
    mode_switches: usize,
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
impl OrbitCharacteristics {
    /// Creates a new `OrbitCharacteristics` instance using data from a provided closed orbit
    /// and a flight computer.
    ///
    /// # Arguments
    /// - `c_orbit`: A reference to the `ClosedOrbit` to derive orbital parameters.
    /// - `f_cont`: A reference to a thread-safe, asynchronous flight computer instance.
    ///
    /// # Returns
    /// A new `OrbitCharacteristics` instance.
    ///
    /// # Panics
    /// This function will panic if the `ClosedOrbit`'s period cannot be converted to an
    /// `usize` or `i64`.
    pub async fn new(c_orbit: &ClosedOrbit, f_cont: &RwLock<FlightComputer>) -> Self {
        let img_dt = c_orbit.max_image_dt();
        let orbit_full_period = c_orbit.period().0.to_num::<usize>();
        let i_entry =
            IndexedOrbitPosition::new(0, orbit_full_period, f_cont.read().await.current_pos());
        Self { img_dt, orbit_full_period, i_entry, mode_switches: 0}
    }

    /// Retrieves the maximum image capture time interval.
    pub fn img_dt(&self) -> I32F32 { self.img_dt }

    /// Retrieves the full orbital period.
    pub fn orbit_full_period(&self) -> usize { self.orbit_full_period }

    /// Retrieves the indexed entry position of the current orbit entry.
    pub fn i_entry(&self) -> IndexedOrbitPosition { self.i_entry }

    /// Returns the number of mode switches already performed
    pub fn mode_switches(&self) -> usize { self.mode_switches }
    /// Marks the end of an orbital mode and updates the entry position.
    ///
    /// # Arguments
    /// - `now`: The new `IndexedOrbitPosition` representing the current state.
    pub fn finish(&mut self, now_pos: Vec2D<I32F32>, rationale: &str) {
        let now = self.i_entry.new_from_pos(now_pos);
        info!(
            "Finished Phase after: {}s, due to: {rationale}",
            (now.t() - self.i_entry.t()).num_seconds()
        );
        self.i_entry = now;
        self.mode_switches += 1;
    }

    /// Marks the end of an orbital mode after re-entering the orbit.
    ///
    /// # Arguments
    /// * `now_pos`: The current position
    /// * `index`: The return index where re-entering was performed 
    pub fn finish_entry(&mut self, now_pos: Vec2D<I32F32>, index: usize) {
        let now = IndexedOrbitPosition::new(index, self.orbit_full_period, now_pos);
        info!(
            "Finished Phase after: {}s, due to: Orbit Reentry",
            (now.t() - self.i_entry.t()).num_seconds()
        );
        self.i_entry = now;
        self.mode_switches += 1;
    }
}
