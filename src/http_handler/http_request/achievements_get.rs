use super::request_common::HTTPRequestType;
use super::achievements::AchievementsResponse;

#[derive(serde::Serialize)]
struct AchievementsRequest{}

impl HTTPRequestType for AchievementsRequest {
    type Response = AchievementsResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/achievements" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}