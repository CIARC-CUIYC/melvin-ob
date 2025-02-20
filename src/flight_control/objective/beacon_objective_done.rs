use crate::flight_control::objective::beacon_objective::BeaconObjective;
use crate::http_handler::http_client::HTTPClient;
use std::sync::Arc;
use fixed::types::I32F32;
use crate::flight_control::common::vec2d::Vec2D;

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
    pub async fn guess_max(&self, client: Arc<HTTPClient>) { 
        todo!() 
    }
}

impl From<BeaconObjective> for BeaconObjectiveDone {
    fn from(obj: BeaconObjective) -> Self {
        let guesses = if let Some(meas) = obj.measurements() {
            meas.pack_perfect_circles()
        } else {
            vec![]
        };
        Self {
            id: obj.id(),
            name: String::from(obj.name()),
            start: obj.start(),
            end: obj.end(),
            guesses,
        }
    }
}
