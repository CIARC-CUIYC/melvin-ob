use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::restore_backup;

#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct RestoreBackupRequest {}

impl NoBodyHTTPRequestType for RestoreBackupRequest {}

impl HTTPRequestType for RestoreBackupRequest {
    type Response = restore_backup::RestoreBackupResponse;
    fn endpoint(&self) -> &str {
        "/backup"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Put
    }
}
