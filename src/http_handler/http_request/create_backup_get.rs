use super::request_common::{HTTPRequest, HTTPRequestType};
use super::create_backup::CreateBackupResponse;

#[derive(serde::Serialize)]
pub struct CreateBackupRequest {}

impl Into<HTTPRequest<Self>> for CreateBackupRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Get(self)
    }
}

impl HTTPRequestType for CreateBackupRequest {
    type Response = CreateBackupResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/backup" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}