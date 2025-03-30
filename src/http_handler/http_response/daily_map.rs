use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

/// Response type for the /dailyMap endpoint
pub(crate) struct DailyMapResponse {}

impl JSONBodyHTTPResponseType for DailyMapResponse {}

impl HTTPResponseType for DailyMapResponse {
    /// Parsed type of the response from this endpoint
    type ParsedResponseType = String;

    /// reads and parses the json response
    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        Self::parse_json_body(resp).await
    }
}
