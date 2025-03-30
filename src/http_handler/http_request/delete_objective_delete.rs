use super::delete_objective;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use std::collections::HashMap;

/// Request type for the /objective endpoint -> DELETE.
#[derive(Debug)]
#[cfg(debug_assertions)]
pub(crate) struct DeleteObjectiveRequest {
    /// The id of the objective to be deleted.
    id: usize,
}

impl NoBodyHTTPRequestType for DeleteObjectiveRequest {}

impl HTTPRequestType for DeleteObjectiveRequest {
    /// Type of the expected response.
    type Response = delete_objective::DeleteObjectiveResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/objective" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Delete }
    /// A `HashMap` containing the queary param key value pairs.
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert("id", self.id.to_string());
        query
    }
}
