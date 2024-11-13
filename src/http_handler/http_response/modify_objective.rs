use crate::http_handler::http_response::response_common::{HTTPResponseType, 
                                                          JSONBodyHTTPResponseType, ResponseError};

// TODO: 422 Response Code: Validation Error -> not implemented

#[cfg(debug_assertions)]
#[derive(serde::Deserialize, Debug)]
pub struct ModifyObjectiveResponse{
    added: Vec<usize>,
    modified: Vec<usize>,
}

impl JSONBodyHTTPResponseType for ModifyObjectiveResponse{}

impl HTTPResponseType for ModifyObjectiveResponse{
    type ParsedResponseType = Self;

    async fn read_response(response: reqwest::Response) 
        -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response)?;
        Ok(Self::parse_json_body(response).await?)
    }
}

