use super::beacon_objective::BeaconObjective;
use crate::flight_control::{common::vec2d::Vec2D, flight_computer::FlightComputer};
use crate::http_handler::{
    http_client::HTTPClient,
    http_request::{
        beacon_position_put::BeaconPositionRequest, request_common::NoBodyHTTPRequestType,
    },
};
use crate::{error, obj};
use chrono::{DateTime, Utc};
use fixed::types::I32F32;
use std::sync::Arc;

#[derive(Clone)]
pub struct BeaconObjectiveDone {
    id: usize,
    name: String,
    start: DateTime<Utc>,
    end: DateTime<Utc>,
    guesses: Vec<Vec2D<I32F32>>,
    submitted: bool,
}

impl BeaconObjectiveDone {
    pub fn id(&self) -> usize { self.id }
    pub fn name(&self) -> &str { &self.name }
    pub fn start(&self) -> DateTime<Utc> { self.start }
    pub fn end(&self) -> DateTime<Utc> { self.end }
    pub fn guesses(&self) -> &Vec<Vec2D<I32F32>> { &self.guesses }
    pub fn submitted(&self) -> bool { self.submitted }

    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    pub async fn guess_max(&mut self, client: Arc<HTTPClient>) {
        obj!(
            "Guessing max for {}: {} guesses...",
            self.id,
            self.guesses.len()
        );
        self.submitted = true;
        let id_u16 = self.id() as u16;
        'outer: for (i, guess) in self.guesses().iter().enumerate() {
            let width = guess.x().abs().to_num::<u32>();
            let height = guess.y().abs().to_num::<u32>();
            let req = BeaconPositionRequest { beacon_id: id_u16, width, height };
            obj!("Sending request for beacon {id_u16} with width {width} and height {height}...");
            loop {
                if let Ok(msg) = req.send_request(&client).await {
                    if msg.is_success() {
                        obj!("Found beacon {id_u16} at {guess}!");
                        return;
                    } else if msg.is_last() {
                        obj!("Could not find beacon {id_u16} after {i} tries!");
                        return;
                    } else if msg.is_unknown() {
                        obj!("Beacon {id_u16} is unknown!");
                        return;
                    } else if msg.is_fail() {
                        continue 'outer;
                    }
                    obj!("Unknown Message: {}! Returning!", msg.msg());
                    return;
                }
                error!("Unnoticed HTTP Error in updateObservation()");
                tokio::time::sleep(FlightComputer::STD_REQUEST_DELAY).await;
            }
        }
    }
}

impl From<BeaconObjective> for BeaconObjectiveDone {
    fn from(obj: BeaconObjective) -> Self {
        let guesses =
            if let Some(meas) = obj.measurements() { meas.pack_perfect_circles() } else { vec![] };
        Self {
            id: obj.id(),
            name: String::from(obj.name()),
            start: obj.start(),
            end: obj.end(),
            guesses,
            submitted: false,
        }
    }
}
