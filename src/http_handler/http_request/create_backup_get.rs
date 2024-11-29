use super::create_backup;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct CreateBackupRequest {}

impl NoBodyHTTPRequestType for CreateBackupRequest {}

impl HTTPRequestType for CreateBackupRequest {
    type Response = create_backup::CreateBackupResponse;
    fn endpoint(&self) -> &str {
        "/backup"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Get
    }
}
