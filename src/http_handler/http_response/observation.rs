use crate::http_handler::http_response::response_common::{HTTPResponseType, JSONBodyHTTPResponseType, ResponseError, SerdeJSONBodyHTTPResponseType};

#[derive(serde::Deserialize, Debug)]
pub struct ObservationResponse {
    state: String,
    angle: String,
    simulation_speed: u32,
    width_x: f64,
    height_y: f64,
    vx: f64,
    vy: f64,
    battery: f32,
    max_battery: f32,
    fuel: f32,
    distance_covered: f64,
    area_covered: AreaCoveredByLens,
    data_volume: DataVolume,
    images_taken: u16,
    active_time: u32,
    objectives_done: u8,
    objectives_points: u16,
    timestamp: chrono::DateTime<chrono::Utc>,
}

impl SerdeJSONBodyHTTPResponseType for ObservationResponse {}

impl ObservationResponse {
    pub fn state(&self) -> &str { self.state.as_str() }
    pub fn angle(&self) -> &str { self.angle.as_str() }
    pub fn simulation_speed(&self) -> u32 { self.simulation_speed }
    pub fn pos_x(&self) -> f64 { self.width_x }
    pub fn pos_y(&self) -> f64 { self.height_y }
    pub fn vel_x(&self) -> f64 { self.vx }
    pub fn vel_y(&self) -> f64 { self.vy }
    pub fn battery(&self) -> f32 { self.battery }
    pub fn max_battery(&self) -> f32 { self.max_battery }
    pub fn fuel(&self) -> f32 { self.fuel }
    pub fn timestamp(&self) -> chrono::DateTime<chrono::Utc> { self.timestamp }
}


#[derive(serde::Deserialize, Debug)]
pub struct AreaCoveredByLens {
    narrow: f32,
    normal: f32,
    wide: f32,
}

#[derive(serde::Deserialize, Debug)]
pub struct DataVolume {
    data_volume_sent: u32,
    data_volume_received: u32,
}