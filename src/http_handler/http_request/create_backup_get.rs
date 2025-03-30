use super::create_backup;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

/// Request type for the /backup endpoint -> GET.
#[derive(Debug)]
#[cfg(debug_assertions)]
pub(crate) struct CreateBackupRequest {}

impl NoBodyHTTPRequestType for CreateBackupRequest {}

impl HTTPRequestType for CreateBackupRequest {
    /// Type of the expected response.
    type Response = create_backup::CreateBackupResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/backup" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
