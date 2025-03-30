use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

/// Response type for the /backup endpoint -> GET
#[cfg(debug_assertions)]
pub(crate) struct CreateBackupResponse {}

impl JSONBodyHTTPResponseType for CreateBackupResponse {}

impl HTTPResponseType for CreateBackupResponse {
    /// The parsed type of the response.
    type ParsedResponseType = String;

    /// reads the HTTP Response Json Body
    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        Self::parse_json_body(resp).await
    }
}
