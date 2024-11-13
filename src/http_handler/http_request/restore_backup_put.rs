use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::restore_backup::RestoreBackupResponse;

#[cfg(debug_assertions)]
#[derive(Debug)]
pub struct RestoreBackupRequest {}

impl NoBodyHTTPRequestType for RestoreBackupRequest {}

impl HTTPRequestType for RestoreBackupRequest {
    type Response = RestoreBackupResponse;
    fn endpoint(&self) -> &str { "/backup" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
}