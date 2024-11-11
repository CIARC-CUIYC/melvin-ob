use super::request_common::{HTTPRequest, HTTPRequestType};
use super::achievements::AchievementsResponse;

#[derive(serde::Serialize)]
pub struct AchievementsRequest{}

impl Into<HTTPRequest<Self>> for AchievementsRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Get(self)
    }
}

impl HTTPRequestType for AchievementsRequest {
    type Response = AchievementsResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/achievements" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}