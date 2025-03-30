use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::reset;

/// Request type for the /reset endpoint.
#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct ResetRequest {}

impl NoBodyHTTPRequestType for ResetRequest {}

impl HTTPRequestType for ResetRequest {
    /// Type of the expected response.
    type Response = reset::ResetResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/reset" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
