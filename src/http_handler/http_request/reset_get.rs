use super::request_common::{HTTPRequest, HTTPRequestType};
use super::reset::ResetResponse;

#[cfg(debug_assertions)]
#[derive(serde::Serialize)]
pub struct ResetRequest {}

impl Into<HTTPRequest<Self>> for ResetRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Get(self)
    }
}

impl HTTPRequestType for ResetRequest {
    type Response = ResetResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/reset" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}