use super::request_common::{HTTPRequest, HTTPRequestType};
use super::restore_backup::RestoreBackupResponse;

#[cfg(debug_assertions)]
#[derive(serde::Serialize, Debug)]
pub struct RestoreBackupRequest {}

impl Into<HTTPRequest<Self>> for RestoreBackupRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Put(self)
    }
}

impl HTTPRequestType for RestoreBackupRequest {
    type Response = RestoreBackupResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/backup" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}