use super::request_common::HTTPRequestType;
use super::delete_objective::DeleteObjectiveResponse;

#[derive(serde::Serialize)]
struct DeleteObjectiveRequest {
    id: usize
}

impl HTTPRequestType for DeleteObjectiveRequest {
    type Response = DeleteObjectiveResponse;
    type Body = DeleteObjectiveRequest;
    fn endpoint(&self) -> &str { "/objective" }
    fn body(&self) -> &Self::Body { self }
}