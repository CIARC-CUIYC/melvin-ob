use crate::http_handler::serde;

#[derive(serde::Deserialize)]
pub struct ObjectiveListResponse{
    zoned_objectives: Vec<ZonedObjectives>,
    beacon_objectives: Vec<BeaconObjectives>
}

#[derive(serde::Deserialize)]
struct ZonedObjectives{
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
struct BeaconObjectives{
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
}