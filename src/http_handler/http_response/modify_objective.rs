use crate::http_handler::http_response::response_common::{HTTPResponseType, JSONBodyHTTPResponseType, ResponseError, SerdeJSONBodyHTTPResponseType};

#[cfg(debug_assertions)]
#[derive(serde::Deserialize, Debug)]
pub struct ModifyObjectiveResponse{
    added: Vec<usize>,
    modified: Vec<usize>,
}

impl SerdeJSONBodyHTTPResponseType for ModifyObjectiveResponse{}

