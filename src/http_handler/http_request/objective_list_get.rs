use super::request_common::{HTTPRequest, HTTPRequestType};
use super::objective_list::ObjectiveListResponse;

#[derive(serde::Serialize, Debug)]
pub struct ObjectiveListRequest {}

impl Into<HTTPRequest<Self>> for ObjectiveListRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Get(self)
    }
}

impl HTTPRequestType for ObjectiveListRequest {
    type Response = ObjectiveListResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/objective" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}