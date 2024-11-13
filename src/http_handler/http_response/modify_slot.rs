use crate::http_handler::http_response::response_common::{HTTPResponseType, JSONBodyHTTPResponseType, ResponseError};

// TODO: 422 Response Code: Validation Error -> not implemented

#[derive(serde::Deserialize, Debug)]
pub struct ModifySlotResponse {
    id: usize,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    enabled: bool,
}

impl JSONBodyHTTPResponseType for ModifySlotResponse {}

impl HTTPResponseType for ModifySlotResponse {
    type ParsedResponseType = Self;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response)?;
        Ok(Self::parse_json_body(response).await?)
    }
}
