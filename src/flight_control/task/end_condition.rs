use crate::flight_control::flight_state::FlightState;
use crate::flight_control::orbit::BurnSequence;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;

pub struct EndCondition {
    end_charge: I32F32,
    end_state: FlightState,
    end_time: DateTime<Utc>,
}

impl EndCondition {
    pub fn from_burn(burn: &BurnSequence) -> Self {
        Self {
            end_time: burn.start_i().t(),
            end_charge: burn.min_charge(),
            end_state: FlightState::Acquisition,
        }
    }
    
    pub fn end_time(&self) -> DateTime<Utc> {self.end_time}
    pub fn end_charge(&self) -> I32F32 {self.end_charge}
    pub fn end_state(&self) -> FlightState {self.end_state}
    
    pub fn min_charge_time(&self) -> TimeDelta {
        let secs = (self.end_charge() / FlightState::Charge.get_charge_rate()).round().to_num::<i64>();
        TimeDelta::seconds(secs)
    }
}
