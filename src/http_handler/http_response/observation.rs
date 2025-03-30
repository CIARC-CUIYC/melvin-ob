use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;
use chrono::{DateTime, Utc};

/// Response type for the /observation endpoint
#[derive(serde::Deserialize, Debug)]
pub(crate) struct ObservationResponse {
    /// The current `FlightState` encoded as a string (e.g. "acquisition" or "safe").
    state: String,
    /// The current `CameraAngle` encoded as a string (e.g. "normal" or "narrow").
    angle: String,
    /// The current simulation speed factor.
    simulation_speed: u32,
    /// The current position on the x-axis.
    width_x: u16,
    /// The current position on the y-axis.
    height_y: u16,
    /// The current velocity in x-direction.
    vx: f64,
    /// The current velocity in y-direction.
    vy: f64,
    /// The current charge level of the battery.
    battery: f64,
    /// The current maximum charge level of the battery.
    max_battery: f64,
    /// The amount of fuel thats left.
    fuel: f64,
    /// The distance MELVIN already covered.
    distance_covered: f64,
    /// The mapping coverage per lens.
    area_covered: AreaCoveredByLens,
    /// The received and sent data volume.
    data_volume: DataVolume,
    /// The number of images that were already shot.
    images_taken: u32,
    /// The time melvin has been active.
    active_time: f64,
    /// The number of objectives that are already finished.
    objectives_done: u16,
    /// The number of points that were awarded for the finished objectives.
    objectives_points: u32,
    /// The current UTC Timestamp.
    timestamp: DateTime<Utc>,
}

impl SerdeJSONBodyHTTPResponseType for ObservationResponse {}

impl ObservationResponse {
    /// Returns the current flight state as a string.
    pub(crate) fn state(&self) -> &str { self.state.as_str() }
    /// Returns the current camera lens as a string.
    pub(crate) fn angle(&self) -> &str { self.angle.as_str() }
    /// Returns the current simulation speed factor.
    pub(crate) fn simulation_speed(&self) -> u32 { self.simulation_speed }
    /// Returns the current position on the x-axis.
    pub(crate) fn pos_x(&self) -> u16 { self.width_x }
    /// Returns the current position on the y-axis.
    pub(crate) fn pos_y(&self) -> u16 { self.height_y }
    /// Returns the velocity in x-direction.
    pub(crate) fn vel_x(&self) -> f64 { self.vx }
    /// Returns the velocity in y-direction.
    pub(crate) fn vel_y(&self) -> f64 { self.vy }
    /// Returns the current battery level.
    pub(crate) fn battery(&self) -> f64 { self.battery }
    /// Returns the current maximum battery level.
    pub(crate) fn max_battery(&self) -> f64 { self.max_battery }
    /// Returns the remaining amount of fuel.
    pub(crate) fn fuel(&self) -> f64 { self.fuel }
    /// Returns the current timestamp in UTC.
    pub(crate) fn timestamp(&self) -> DateTime<Utc> { self.timestamp }
}

/// Struct holding coverage information per camera lens
#[derive(serde::Deserialize, Debug)]
pub(crate) struct AreaCoveredByLens {
    /// The coverage from `CameraAngle::Narrow`.
    narrow: f64,
    /// The coverage from `CameraAngle::Normal`.
    normal: f64,
    /// The coverage from `CameraAngle::Wide`.
    wide: f64,
}

/// Struct containing information on the received and sent number of bytes
#[derive(serde::Deserialize, Debug)]
pub(crate) struct DataVolume {
    /// Number of bytes that were already sent.
    data_volume_sent: u32,
    /// Number of bytes that were already received
    data_volume_received: u32,
}
