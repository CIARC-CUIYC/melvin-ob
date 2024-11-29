use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

#[cfg(debug_assertions)]
pub struct RestoreBackupResponse {}

impl JSONBodyHTTPResponseType for RestoreBackupResponse {}

impl HTTPResponseType for RestoreBackupResponse {
    type ParsedResponseType = String;

    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Ok(Self::parse_json_body(response).await?)
    }
}
