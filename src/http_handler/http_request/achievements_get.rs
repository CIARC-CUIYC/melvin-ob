use super::achievements::AchievementsResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

#[derive(Debug)]
pub struct AchievementsRequest {}

impl NoBodyHTTPRequestType for AchievementsRequest {}

impl HTTPRequestType for AchievementsRequest {
    type Response = AchievementsResponse;
    fn endpoint(&self) -> &'static str { "/achievements" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
