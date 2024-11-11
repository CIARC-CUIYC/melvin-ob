use super::request_common::{HTTPRequest, HTTPRequestType};
use super::delete_objective::DeleteObjectiveResponse;

#[cfg(debug_assertions)]
#[derive(serde::Serialize)]
pub struct DeleteObjectiveRequest {
    id: usize,
}

impl Into<HTTPRequest<Self>> for DeleteObjectiveRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Delete(self)
    }
}

impl HTTPRequestType for DeleteObjectiveRequest {
    type Response = DeleteObjectiveResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/objective" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.append("id", self.id.into());
        headers
    }
}