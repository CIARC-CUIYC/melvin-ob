use fixed::types::{I32F32, I64F64};
use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

#[derive(serde::Deserialize, Debug)]
pub struct ObservationResponse {
    state: String,
    angle: String,
    simulation_speed: u32,
    width_x: I32F32,
    height_y: I32F32,
    vx: I32F32,
    vy: I32F32,
    battery: I32F32,
    max_battery: I32F32,
    fuel: I32F32,
    distance_covered: I64F64,
    area_covered: AreaCoveredByLens,
    data_volume: DataVolume,
    images_taken: u32,
    active_time: I64F64,
    objectives_done: u16,
    objectives_points: u32,
    timestamp: chrono::DateTime<chrono::Utc>,
}

impl SerdeJSONBodyHTTPResponseType for ObservationResponse {}

impl ObservationResponse {
    pub fn state(&self) -> &str { self.state.as_str() }
    pub fn angle(&self) -> &str { self.angle.as_str() }
    pub fn simulation_speed(&self) -> u32 { self.simulation_speed }
    pub fn pos_x(&self) -> I32F32 { self.width_x }
    pub fn pos_y(&self) -> I32F32 { self.height_y }
    pub fn vel_x(&self) -> I32F32 { self.vx }
    pub fn vel_y(&self) -> I32F32 { self.vy }
    pub fn battery(&self) -> I32F32 { self.battery }
    pub fn max_battery(&self) -> I32F32 { self.max_battery }
    pub fn fuel(&self) -> I32F32 { self.fuel }
    pub fn timestamp(&self) -> chrono::DateTime<chrono::Utc> { self.timestamp }
}

#[derive(serde::Deserialize, Debug)]
pub struct AreaCoveredByLens {
    narrow: I32F32,
    normal: I32F32,
    wide: I32F32,
}

#[derive(serde::Deserialize, Debug)]
pub struct DataVolume {
    data_volume_sent: u32,
    data_volume_received: u32,
}
