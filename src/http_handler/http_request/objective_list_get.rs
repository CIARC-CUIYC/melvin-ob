use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::objective_list::ObjectiveListResponse;

#[derive(Debug)]
pub struct ObjectiveListRequest {}

impl NoBodyHTTPRequestType for ObjectiveListRequest {}

impl HTTPRequestType for ObjectiveListRequest {
    type Response = ObjectiveListResponse;
    fn endpoint(&self) -> &str { "/objective" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
}