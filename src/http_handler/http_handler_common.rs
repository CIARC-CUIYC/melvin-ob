use crate::http_handler::serde;

#[derive(serde::Deserialize)]
pub struct ZonedObjective{
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    decrease_rate: i64,
    enabled: bool,
    zone: [i32; 4],
    // TODO: make an enum out of optic_required
    optic_required: String,
    coverage_required: usize,
    description: String,
    sprite: String,
    secret: bool
}

#[derive(serde::Deserialize)]
pub struct BeaconObjective{
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
}

#[derive(serde::Deserialize)]
pub struct CommunicationSlot{
    id: usize,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    enabled: bool
}