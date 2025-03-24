use crate::flight_control::flight_state::FlightState;
use crate::flight_control::orbit::BurnSequence;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;

pub struct EndCondition {
    charge: I32F32,
    state: FlightState,
    time: DateTime<Utc>,
}

impl EndCondition {
    pub fn from_burn(burn: &BurnSequence) -> Self {
        Self {
            time: burn.start_i().t(),
            charge: burn.min_charge(),
            state: FlightState::Acquisition,
        }
    }

    pub fn time(&self) -> DateTime<Utc> { self.time }
    pub fn charge(&self) -> I32F32 { self.charge }
    pub fn state(&self) -> FlightState { self.state }

    pub fn abs_charge_dt(&self) -> TimeDelta {
        let secs = (self.charge() / FlightState::Charge.get_charge_rate()).round().to_num::<i64>();
        TimeDelta::seconds(secs)
    }
}
