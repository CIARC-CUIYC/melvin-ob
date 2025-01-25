use super::delete_objective;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use std::collections::HashMap;

#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct DeleteObjectiveRequest {
    id: usize,
}

impl NoBodyHTTPRequestType for DeleteObjectiveRequest {}

impl HTTPRequestType for DeleteObjectiveRequest {
    type Response = delete_objective::DeleteObjectiveResponse;
    fn endpoint(&self) -> &'static str { "/objective" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Delete }
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert("id", self.id.to_string());
        query
    }
}
