use crate::http_handler::http_handler_common::Achievement;
use super::response_common::{HTTPResponseType, ResponseError, JSONBodyHTTPResponseType, SerdeJSONBodyHTTPResponseType};

#[derive(serde::Deserialize, Debug)]
pub struct AchievementsResponse {
    achievements: Vec<Achievement>
}

impl SerdeJSONBodyHTTPResponseType for AchievementsResponse {}