use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

/// Response type for the /objective endpoint -> PUT
#[cfg(debug_assertions)]
#[derive(serde::Deserialize, Debug)]
pub(crate) struct ModifyObjectiveResponse {
    /// The indices that were added to the objective list
    added: Vec<usize>,
    /// The indices from the objective list that were modified
    modified: Vec<usize>,
}

impl SerdeJSONBodyHTTPResponseType for ModifyObjectiveResponse {}
