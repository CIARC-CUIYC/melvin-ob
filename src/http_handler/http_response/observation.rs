use crate::http_handler::http_response::response_common::{HTTPResponseType,
                                                          JSONBodyHTTPResponseType, ResponseError};

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

impl JSONBodyHTTPResponseType for ObservationResponse {}

impl HTTPResponseType for ObservationResponse {
    type ParsedResponseType = Self;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Ok(Self::parse_json_body(response).await?)
    }
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