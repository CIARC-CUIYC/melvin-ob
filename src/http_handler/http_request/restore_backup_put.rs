use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::restore_backup;

/// Request type for the /backup endpoint -> PUT.
#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct RestoreBackupRequest {}

impl NoBodyHTTPRequestType for RestoreBackupRequest {}

impl HTTPRequestType for RestoreBackupRequest {
    /// Type of the expected response.
    type Response = restore_backup::RestoreBackupResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/backup" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
}
