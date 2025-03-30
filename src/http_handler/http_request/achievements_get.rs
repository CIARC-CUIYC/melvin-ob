use super::achievements::AchievementsResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

/// Request type for the /achievements endpoint.
#[derive(Debug)]
pub struct AchievementsRequest {}

impl NoBodyHTTPRequestType for AchievementsRequest {}

impl HTTPRequestType for AchievementsRequest {
    /// Type of the expected response.
    type Response = AchievementsResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/achievements" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
