use crate::http_handler::http_response::response_common::{HTTPResponseType,
                                                          JSONBodyHTTPResponseType, ResponseError};

// TODO: 422 Response Code: Validation Error -> not implemented

#[cfg(debug_assertions)]
pub struct ConfigureSimulationResponse {}

impl JSONBodyHTTPResponseType for ConfigureSimulationResponse {}

impl HTTPResponseType for ConfigureSimulationResponse {
    type ParsedResponseType = String;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Ok(Self::parse_json_body(response).await?)
    }
}

