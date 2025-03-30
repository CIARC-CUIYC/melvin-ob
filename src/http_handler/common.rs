use super::http_request::request_common::RequestError;
use super::http_response::response_common::ResponseError;
use chrono::{DateTime, Utc};
use strum_macros::Display;

/// Represents the different types of imaging objective zones.
///
/// A zone can either be:
/// - `KnownZone`: a rectangular area defined by four integers `[x_1, y_1, x_2, y_2]`.
/// - `SecretZone`: a zone with unknown dimensions and an unknown position.
#[derive(serde::Deserialize, serde::Serialize, Debug, Clone)]
#[serde(untagged)]
pub(crate) enum ZoneType {
    /// A rectangular zone defined by `[x, y, x_2, y_2]`.
    KnownZone([i32; 4]),
    /// A secret zone.
    SecretZone(String),
}

/// Describes an imaging objective with a defined timeframe, required coverage,
/// and optical constraints.
///
/// These objectives are fulfilled by imaging specific map areas (zones) using a satellite camera.
#[derive(serde::Deserialize, serde::Serialize, Debug, Clone)]
pub(crate) struct ImageObjective {
    /// Unique identifier for the objective.
    id: usize,
    /// Human-readable name of the objective.
    name: String,
    /// UTC timestamp marking when the objective becomes active.
    start: DateTime<Utc>,
    /// UTC timestamp after which the objective can no longer be fulfilled.
    end: DateTime<Utc>,
    /// Unused parameter.
    decrease_rate: f32,
    /// The target zone type: either known or secret.
    zone: ZoneType,
    /// Required camera lens configuration (e.g., `"wide"` or `"narrow"`).
    optic_required: String,
    /// Minimum percentage of zone coverage required for success (range 0.0–1.0).
    coverage_required: f64,
    /// Unused parameter.
    sprite: Option<String>,
    /// Whether the objective is secret or not.
    secret: bool,
}

impl ImageObjective {
    /// Returns the objective’s unique identifier.
    pub(crate) fn id(&self) -> usize { self.id }
    /// Returns the UTC start time for the objective.
    pub(crate) fn start(&self) -> DateTime<Utc> { self.start }
    /// Returns the UTC end time for the objective.
    pub(crate) fn end(&self) -> DateTime<Utc> { self.end }
    /// Returns the objective's display name.
    pub(crate) fn name(&self) -> &str { &self.name }
    /// Returns the associated zone type (known or secret).
    pub(crate) fn zone_type(&self) -> &ZoneType { &self.zone }
    /// Returns the required optical configuration (e.g. "wide", "narrow").
    pub(crate) fn optic_required(&self) -> &str { &self.optic_required }
    /// Returns the minimum coverage (fraction) required to fulfill the objective.
    pub(crate) fn coverage_required(&self) -> f64 { self.coverage_required }
    /// Returns whether the objective is secret.
    pub(crate) fn is_secret(&self) -> bool { self.secret }
}

/// A mission objective involving beacon detection and signal noise filtering.
#[derive(serde::Deserialize, serde::Serialize, Debug, Clone)]
pub struct BeaconObjective {
    /// Unique identifier for the beacon objective.
    id: usize,
    /// Display name of the beacon objective.
    name: String,
    /// UTC start time for the valid measurement window.
    start: DateTime<Utc>,
    /// UTC end time for the valid measurement window.
    end: DateTime<Utc>,
    /// Unused parameter.
    decrease_rate: f32,
    /// Number of attempts already made for this objective.
    attempts_made: u32,
    /// Description and contextual details about the objective.
    description: String,
}

impl BeaconObjective {
    /// Returns the beacon objective’s name.
    pub(crate) fn name(&self) -> &str { self.name.as_str() }
    /// Returns the unique ID of the objective.
    pub(crate) fn id(&self) -> usize { self.id }
    /// Returns the number of attempts made to fulfill the objective.
    pub(crate) fn attempts_made(&self) -> u32 { self.attempts_made }
    /// Returns the UTC start time of the objective.
    pub(crate) fn start(&self) -> DateTime<Utc> { self.start }
    /// Returns the UTC end time of the objective.
    pub(crate) fn end(&self) -> DateTime<Utc> { self.end }
    /// Returns the human-readable objective description.
    pub(crate) fn description(&self) -> &str { self.description.as_str() }
}

/// A time slot during which communication (e.g., console downlink) is enabled.
#[derive(serde::Deserialize, Debug)]
pub struct CommunicationSlot {
    /// Unique ID of the communication slot.
    id: usize,
    /// UTC timestamp when the slot opens.
    start: DateTime<Utc>,
    /// UTC timestamp when the slot closes.
    end: DateTime<Utc>,
    /// Whether communication is enabled for this slot.
    enabled: bool,
}

impl CommunicationSlot {
    /// Returns whether this communication slot is currently enabled.
    fn is_enabled(&self) -> bool { self.enabled }

    /// Returns the unique identifier for this slot.
    fn id(&self) -> usize { self.id }
}

/// Represents an achievement milestone defined by the simulation backend.
#[derive(serde::Deserialize, Debug)]
pub struct Achievement {
    /// Unique name or label of the achievement.
    name: String,
    /// Whether the achievement has been completed.
    done: bool,
    /// Number of points awarded for completing the achievement.
    points: f32,
    /// A detailed explanation of the achievement goal.
    description: String,
    /// Indicates whether the parameter threshold was crossed.
    goal_parameter_threshold: bool,
    /// Indicates whether the achievement's goal condition is satisfied.
    goal_parameter: bool,
}

impl Achievement {
    /// Returns the name of the achievement.
    fn name(&self) -> &str { &self.name }
    /// Returns whether the achievement has been completed.
    fn is_done(&self) -> bool { self.done }
    /// Returns the number of points this achievement is worth.
    fn points(&self) -> f32 { self.points }
    /// Returns the textual description of this achievement.
    fn description(&self) -> &str { &self.description }
    /// Returns whether the goal threshold has been reached.
    fn is_goal_parameter_threshold(&self) -> bool { self.goal_parameter_threshold }
    /// Returns whether the goal parameter is currently satisfied.
    fn is_goal_parameter(&self) -> bool { self.goal_parameter }
}

/// High-level error type that wraps both HTTP request and response errors.
///
/// Used throughout the API and backend interface layers to propagate errors between
/// request and response handlers.
#[derive(Debug, Display)]
pub enum HTTPError {
    /// Represents a failure during the HTTP request phase.
    HTTPRequestError(RequestError),
    /// Represents a failure while parsing or interpreting the HTTP response.
    HTTPResponseError(ResponseError),
}

impl std::error::Error for HTTPError {}
