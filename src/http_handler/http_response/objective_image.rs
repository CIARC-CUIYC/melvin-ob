use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

/// Response type for the /objective endpoint -> POST
pub(crate) struct ObjectiveImageResponse {}

impl JSONBodyHTTPResponseType for ObjectiveImageResponse {}

impl HTTPResponseType for ObjectiveImageResponse {
    /// Type of the parsed response
    type ParsedResponseType = String;

    /// Reads and parses the response from the json body
    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        Self::parse_json_body(resp).await
    }
}
