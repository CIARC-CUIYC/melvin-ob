use crate::http_handler::http_handler_common::Achievement;
use super::response_common::{HTTPResponseType, ResponseError, JSONBodyHTTPResponseType};

#[derive(serde::Deserialize, Debug)]
pub struct AchievementsResponse {
    achievements: Vec<Achievement>
}

impl JSONBodyHTTPResponseType for AchievementsResponse {}

impl HTTPResponseType for AchievementsResponse {
    type ParsedResponseType = Self;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Ok(Self::parse_json_body(response).await?)
    }
}