use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

// TODO: is deserialize possible here? Just a string gets returned

#[cfg(debug_assertions)]
pub struct CreateBackupResponse {}

impl JSONBodyHTTPResponseType for CreateBackupResponse {}

impl HTTPResponseType for CreateBackupResponse {
    type ParsedResponseType = String;

    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Self::parse_json_body(response).await
    }
}
