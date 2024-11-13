use crate::http_handler::http_response::response_common::{HTTPResponseType,
                                                          JSONBodyHTTPResponseType, ResponseError};

// TODO: 422 Response Code: Validation Error -> not implemented

#[cfg(debug_assertions)]
pub struct DeleteObjectiveResponse {}

impl JSONBodyHTTPResponseType for DeleteObjectiveResponse {}

impl HTTPResponseType for DeleteObjectiveResponse {
    type ParsedResponseType = isize;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response)?;
        Ok(Self::parse_json_body(response).await?)
    }
}