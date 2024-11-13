use crate::http_handler::http_response::response_common::{HTTPResponseType, 
                                                          JSONBodyHTTPResponseType, ResponseError};

// TODO: 422 Response Code: Validation Error -> not implemented

pub struct ObjectiveImageResponse {}

impl JSONBodyHTTPResponseType for ObjectiveImageResponse {}

impl HTTPResponseType for ObjectiveImageResponse {
    type ParsedResponseType = String;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response)?;
        Ok(Self::parse_json_body(response).await?)
    }
}
