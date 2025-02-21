use std::cmp::Ordering;
use chrono::TimeDelta;
use fixed::types::I32F32;
use crate::flight_control::common::bayesian_set::BayesianSet;
use crate::flight_control::common::vec2d::{MapSize, Vec2D};
use crate::STATIC_ORBIT_VEL;

#[derive(Debug, Clone)]
pub struct BeaconMeas {
    id: usize,
    pos: Vec2D<I32F32>,
    rssi: f64,
    delay: TimeDelta,
}

impl BeaconMeas {
    pub fn new(id: usize, pos: Vec2D<I32F32>, rssi: f64, delay: TimeDelta) -> Self {
        Self { id, pos, rssi, delay }
    }
    pub fn id(&self) -> usize { self.id }
    pub fn pos(&self) -> &Vec2D<I32F32> { &self.pos }
    pub fn rssi(&self) -> f64 { self.rssi }
    pub fn corr_pos(&self) -> Vec2D<I32F32> {
        let delay_s_fixed = I32F32::from_num(self.delay.num_milliseconds()) / I32F32::from_num(1000);
        (self.pos - Vec2D::from(STATIC_ORBIT_VEL) * delay_s_fixed).wrap_around_map().round()
    }
    pub fn delay(&self) -> TimeDelta { self.delay }
    
}


#[derive(Debug, Clone)]
pub struct BeaconObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    measurements: Option<BayesianSet>
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
            measurements: None,
        }
    }
    pub fn id(&self) -> usize { self.id }
    pub fn name(&self) -> &str { &self.name }
    pub fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    pub fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
    pub fn measurements(&self) -> Option<&BayesianSet> { self.measurements.as_ref() }
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
    fn eq(&self, other: &Self) -> bool { self.end == other.end }
}

impl PartialOrd<Self> for BeaconObjective {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

impl Ord for BeaconObjective {
    fn cmp(&self, other: &Self) -> Ordering { self.end.cmp(&other.end) }
}
