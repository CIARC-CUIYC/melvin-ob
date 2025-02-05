use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

#[cfg(debug_assertions)]
pub struct CreateBackupResponse {}

impl JSONBodyHTTPResponseType for CreateBackupResponse {}

impl HTTPResponseType for CreateBackupResponse {
    type ParsedResponseType = String;

    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        Self::parse_json_body(resp).await
    }
}
