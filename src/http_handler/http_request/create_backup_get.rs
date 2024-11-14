use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::create_backup::CreateBackupResponse;

#[cfg(debug_assertions)]
#[derive(Debug)]
pub struct CreateBackupRequest {}

impl NoBodyHTTPRequestType for CreateBackupRequest {}

impl HTTPRequestType for CreateBackupRequest {
    type Response = CreateBackupResponse;
    fn endpoint(&self) -> &str { "/backup" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}