use crate::STATIC_ORBIT_VEL;
use crate::util::{Vec2D, logger::JsonDump};
use super::BayesianSet;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;
use std::cmp::Ordering;

/// Represents a beacon measurement with associated properties.
#[derive(Debug, Clone, serde::Serialize)]
pub struct BeaconMeas {
    /// Unique identifier of the beacon.
    id: usize,
    /// Position of MELVIN when the beacon measurement was received.
    pos: Vec2D<I32F32>,
    /// Received Signal Strength Indicator (RSSI) or `d_noisy` of the beacon.
    rssi: f64,
    /// Time delay associated with the beacon measurement and the observed position.
    delay: TimeDelta,
}

impl BeaconMeas {
    /// Creates a new [`BeaconMeas`] instance.
    ///
    /// # Arguments
    /// * `id` - Unique identifier of the beacon.
    /// * `pos` - Position of MELVIN as a 2D vector.
    /// * `rssi` - RSSI (`d_noisy`) value of the beacon.
    /// * `delay` - Time delay for the beacon.
    pub fn new(id: usize, pos: Vec2D<I32F32>, rssi: f64, delay: TimeDelta) -> Self {
        Self { id, pos, rssi, delay }
    }

    /// Returns the unique identifier of the beacon.
    pub fn id(&self) -> usize { self.id }
    /// Returns a reference to the position of the beacon.
    pub fn pos(&self) -> &Vec2D<I32F32> { &self.pos }
    /// Returns the RSSI of the beacon.
    pub fn rssi(&self) -> f64 { self.rssi }

    /// Computes and returns the corrected position of MELVIN.
    ///
    /// This considers the delay and adjusts the position accordingly using
    /// `STATIC_ORBIT_VEL`.
    pub fn corr_pos(&self) -> Vec2D<I32F32> {
        let delay_s_fixed =
            I32F32::from_num(self.delay.num_milliseconds()) / I32F32::from_num(1000);
        (self.pos - Vec2D::from(STATIC_ORBIT_VEL) * delay_s_fixed).wrap_around_map().round()
    }

    /// Returns the time delay associated with the beacon measurement.
    pub fn delay(&self) -> TimeDelta { self.delay }
}

/// Represents a beacon objective with associated metadata and measurements.
#[derive(Debug, Clone, serde::Serialize)]
pub struct BeaconObjective {
    /// Unique identifier of the beacon objective.
    id: usize,
    /// Name of the beacon objective.
    name: String,
    /// Start time of the beacon objective.
    start: DateTime<Utc>,
    /// End time of the beacon objective.
    end: DateTime<Utc>,
    /// Optional set of measurements associated with the beacon objective.
    measurements: Option<BayesianSet>,
}

impl JsonDump for BeaconObjective {
    /// Returns the file name for the JSON dump of the beacon objective.
    fn file_name(&self) -> String { format!("bo_{}.json", self.id) }

    /// Returns the directory name for the beacon objective JSON files.
    fn dir_name(&self) -> &'static str { "beacon_objectives" }
}

impl BeaconObjective {
    /// Creates a new [`BeaconObjective`] instance.
    ///
    /// # Arguments
    /// * `id` - Unique identifier of the beacon objective.
    /// * `name` - Name of the beacon objective.
    /// * `start` - Start time of the beacon objective.
    /// * `end` - End time of the beacon objective.
    pub fn new(id: usize, name: String, start: DateTime<Utc>, end: DateTime<Utc>) -> Self {
        Self { id, name, start, end, measurements: None }
    }

    /// Returns the unique identifier of the beacon objective.
    pub fn id(&self) -> usize { self.id }
    /// Returns the name of the beacon objective.
    pub fn name(&self) -> &str { &self.name }
    /// Returns the start time of the beacon objective.
    pub fn start(&self) -> DateTime<Utc> { self.start }
    /// Returns the end time of the beacon objective.
    pub fn end(&self) -> DateTime<Utc> { self.end }
    /// Returns an optional reference to the set of beacon measurements.
    pub fn measurements(&self) -> Option<&BayesianSet> { self.measurements.as_ref() }

    /// Appends a beacon measurement to the objective's measurement set.
    ///
    /// If the measurement set does not exist, it creates a new one.
    ///
    /// # Arguments
    /// * `meas` - The `BeaconMeas` to be added.
    pub fn append_measurement(&mut self, meas: BeaconMeas) {
        if let Some(meas_set) = &mut self.measurements {
            meas_set.update(&meas);
        } else {
            self.measurements = Some(BayesianSet::new(meas));
        }
    }
}

impl Eq for BeaconObjective {}

impl PartialEq<Self> for BeaconObjective {
    /// Compares equality based on the end time of the objectives.
    fn eq(&self, other: &Self) -> bool { self.end == other.end }
}

impl PartialOrd<Self> for BeaconObjective {
    /// Provides partial ordering based on the end time of the objectives.
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

impl Ord for BeaconObjective {
    /// Orders the objectives based on their end time.
    fn cmp(&self, other: &Self) -> Ordering { self.end.cmp(&other.end) }
}

impl From<crate::http_handler::BeaconObjective> for BeaconObjective {
    /// Converts an external `BeaconObjective` into the internal representation.
    ///
    /// # Arguments
    /// * `obj` - The external `BeaconObjective` to be converted.
    fn from(obj: crate::http_handler::BeaconObjective) -> Self {
        Self {
            id: obj.id(),
            name: String::from(obj.name()),
            start: obj.start(),
            end: obj.end(),
            measurements: None,
        }
    }
}
