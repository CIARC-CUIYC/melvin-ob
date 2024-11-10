use super::request_common::HTTPRequestType;
use super::objective_list::ObjectiveListResponse;

struct ObjectiveListRequest {}

impl HTTPRequestType for ObjectiveListRequest {
    type Response = ObjectiveListResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/objective" }
    fn body(&self) -> &Self::Body { &() }
}