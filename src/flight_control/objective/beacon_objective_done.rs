use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::objective::beacon_objective::BeaconObjective;
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_request::beacon_position_put::BeaconPositionRequest;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use fixed::types::I32F32;
use std::sync::Arc;
use crate::{error, obj};

pub struct BeaconObjectiveDone {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    guesses: Vec<Vec2D<I32F32>>,
}

impl BeaconObjectiveDone {
    pub fn id(&self) -> usize { self.id }
    pub fn name(&self) -> &str { &self.name }
    pub fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    pub fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
    pub fn guesses(&self) -> &Vec<Vec2D<I32F32>> { &self.guesses }

    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    pub async fn guess_max(&self, client: Arc<HTTPClient>) {
        let id_u16 = self.id() as u16;
        'outer: for guess in self.guesses() {
            let width = guess.x().abs().to_num::<u32>();
            let height = guess.y().abs().to_num::<u32>();
            let req = BeaconPositionRequest {
                beacon_id: id_u16,
                width,
                height,
            };
            loop {
                if let Ok(msg) = req.send_request(&client).await {
                    if msg.is_success() {
                        obj!("Found beacon {id_u16} at {guess}!");
                        return;
                    } else if msg.is_last() {
                        obj!("Could not find beacon {id_u16}!");
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
        }
    }
}
