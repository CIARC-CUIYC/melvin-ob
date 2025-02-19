use crate::flight_control::objective::beacon_objective::BeaconMeas;
use crate::http_handler::http_client::HTTPClient;
use std::sync::Arc;

pub struct BeaconObjectiveDone {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    attempts_made: usize,
    measurements: Vec<BeaconMeas>,
}

impl BeaconObjectiveDone {
    pub fn id(&self) -> usize { self.id }
    pub fn name(&self) -> &str { &self.name }
    pub fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    pub fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
    pub fn attempts_made(&self) -> usize { self.attempts_made }
    pub fn measurements(&self) -> &Vec<BeaconMeas> { self.measurements.as_ref() }
    pub fn guess_max(&self, client: Arc<HTTPClient>) { todo!() }
}
