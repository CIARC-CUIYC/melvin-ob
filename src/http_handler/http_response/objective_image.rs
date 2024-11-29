use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError,
};

pub struct ObjectiveImageResponse {}

impl JSONBodyHTTPResponseType for ObjectiveImageResponse {}

impl HTTPResponseType for ObjectiveImageResponse {
    type ParsedResponseType = String;

    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Ok(Self::parse_json_body(response).await?)
    }
}
