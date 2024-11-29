use super::delete_objective;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct DeleteObjectiveRequest {
    id: usize,
}

impl NoBodyHTTPRequestType for DeleteObjectiveRequest {}

impl HTTPRequestType for DeleteObjectiveRequest {
    type Response = delete_objective::DeleteObjectiveResponse;
    fn endpoint(&self) -> &str {
        "/objective"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Delete
    }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.append("id", self.id.into());
        headers
    }
}
