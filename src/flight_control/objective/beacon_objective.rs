use std::cmp::Ordering;
use fixed::types::I32F32;
use crate::flight_control::common::vec2d::Vec2D;

#[derive(Debug, Clone)]
pub struct BeaconMeas {
    id: usize,
    pos: Vec2D<I32F32>,
    rssi: f64,
}

impl BeaconMeas {
    pub fn new(id: usize, pos: Vec2D<I32F32>, rssi: f64) -> Self {
        Self { id, pos, rssi }
    }
    pub fn id(&self) -> usize { self.id }
    pub fn pos(&self) -> &Vec2D<I32F32> { &self.pos }
    pub fn rssi(&self) -> f64 { self.rssi }
}


#[derive(Debug, Clone)]
pub struct BeaconObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    attempts_made: usize,
    measurements: Option<Vec<BeaconMeas>>
}

impl BeaconObjective {
    pub fn new(
        id: usize,
        name: String,
        start: chrono::DateTime<chrono::Utc>,
        end: chrono::DateTime<chrono::Utc>,
    ) -> Self {
        Self {
            id,
            name,
            start,
            end,
            attempts_made: 0,
            measurements: None,
        }
    }
    pub fn id(&self) -> usize { self.id }
    pub fn name(&self) -> &str { &self.name }
    pub fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    pub fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
    pub fn measurements(&self) -> Option<&Vec<BeaconMeas>> { self.measurements.as_ref() }
    pub fn attempts_made(&self) -> usize { self.attempts_made }
    pub fn append_measurement(&mut self, meas: BeaconMeas) {
        self.measurements.get_or_insert(Vec::new()).push(meas);
    }
}

impl Eq for BeaconObjective {}

impl PartialEq<Self> for BeaconObjective {
    fn eq(&self, other: &Self) -> bool { self.end == other.end }
}

impl PartialOrd<Self> for BeaconObjective {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

impl Ord for BeaconObjective {
    fn cmp(&self, other: &Self) -> Ordering { self.end.cmp(&other.end) }
}
