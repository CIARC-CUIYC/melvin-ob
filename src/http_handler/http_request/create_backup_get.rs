use super::request_common::HTTPRequestType;
use super::create_backup::CreateBackupResponse;

#[derive(serde::Serialize)]
struct CreateBackupRequest {}

impl HTTPRequestType for CreateBackupRequest {
    type Response = CreateBackupResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/backup" }
    fn body(&self) -> &Self::Body { &() }
}