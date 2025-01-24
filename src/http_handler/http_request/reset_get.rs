use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::reset;

#[derive(Debug)]
#[cfg(debug_assertions)]
pub struct ResetRequest {}

impl NoBodyHTTPRequestType for ResetRequest {}

impl HTTPRequestType for ResetRequest {
    type Response = reset::ResetResponse;
    fn endpoint(&self) -> &'static str {
        "/reset"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Get
    }
}
