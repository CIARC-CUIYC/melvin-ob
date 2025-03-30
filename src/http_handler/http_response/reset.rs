use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

/// Response type for the /reset endpoint
#[cfg(debug_assertions)]
pub(crate) struct ResetResponse {}

impl JSONBodyHTTPResponseType for ResetResponse {}

impl HTTPResponseType for ResetResponse {
    /// Type of the parsed response
    type ParsedResponseType = String;

    /// Reads and parses the response json body 
    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        Self::parse_json_body(resp).await
    }
}
