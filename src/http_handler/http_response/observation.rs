use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

#[derive(serde::Deserialize, Debug)]
pub struct ObservationResponse {
    state: String,
    angle: String,
    simulation_speed: u32,
    width_x: u16,
    height_y: u16,
    vx: f64,
    vy: f64,
    battery: f64,
    max_battery: f64,
    fuel: f64,
    distance_covered: f64,
    area_covered: AreaCoveredByLens,
    data_volume: DataVolume,
    images_taken: u32,
    active_time: f64,
    objectives_done: u16,
    objectives_points: u32,
    timestamp: chrono::DateTime<chrono::Utc>,
}

impl SerdeJSONBodyHTTPResponseType for ObservationResponse {}

impl ObservationResponse {
    pub fn state(&self) -> &str { self.state.as_str() }
    pub fn angle(&self) -> &str { self.angle.as_str() }
    pub fn simulation_speed(&self) -> u32 { self.simulation_speed }
    pub fn pos_x(&self) -> u16 { self.width_x }
    pub fn pos_y(&self) -> u16 { self.height_y }
    pub fn vel_x(&self) -> f64 { self.vx }
    pub fn vel_y(&self) -> f64 { self.vy }
    pub fn battery(&self) -> f64 { self.battery }
    pub fn max_battery(&self) -> f64 { self.max_battery }
    pub fn fuel(&self) -> f64 { self.fuel }
    pub fn timestamp(&self) -> chrono::DateTime<chrono::Utc> { self.timestamp }
}

#[derive(serde::Deserialize, Debug)]
pub struct AreaCoveredByLens {
    narrow: f64,
    normal: f64,
    wide: f64,
}

#[derive(serde::Deserialize, Debug)]
pub struct DataVolume {
    data_volume_sent: u32,
    data_volume_received: u32,
}
