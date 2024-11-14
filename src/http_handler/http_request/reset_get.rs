use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::reset::ResetResponse;

#[cfg(debug_assertions)]
#[derive(Debug)]
pub struct ResetRequest {}

impl NoBodyHTTPRequestType for ResetRequest {}

impl HTTPRequestType for ResetRequest {
    type Response = ResetResponse;
    fn endpoint(&self) -> &str { "/reset" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}