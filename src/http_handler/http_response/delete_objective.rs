use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

/// Response type for the /objective endpoint -> DELETE
#[cfg(debug_assertions)]
pub(crate) struct DeleteObjectiveResponse {}

impl JSONBodyHTTPResponseType for DeleteObjectiveResponse {}

impl HTTPResponseType for DeleteObjectiveResponse {
    /// Parsed type of the response
    type ParsedResponseType = isize;

    /// reads and parses the json response
    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        Self::parse_json_body(resp).await
    }
}
