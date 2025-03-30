use super::objective_list::ObjectiveListResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

/// Request type for the /objective endpoint -> GET.
#[derive(Debug)]
pub(crate) struct ObjectiveListRequest {}

impl NoBodyHTTPRequestType for ObjectiveListRequest {}

impl HTTPRequestType for ObjectiveListRequest {
    /// Type of the expected response.
    type Response = ObjectiveListResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/objective" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
