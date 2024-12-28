use super::response_common::SerdeJSONBodyHTTPResponseType;
use crate::http_handler::http_handler_common::Achievement;

#[derive(serde::Deserialize, Debug)]
pub struct AchievementsResponse {
    achievements: Vec<Achievement>,
}

impl SerdeJSONBodyHTTPResponseType for AchievementsResponse {}
