use super::request_common::HTTPRequestType;
use super::restore_backup::RestoreBackupResponse;

#[derive(serde::Serialize)]
struct RestoreBackupRequest {}

impl HTTPRequestType for RestoreBackupRequest {
    type Response = RestoreBackupResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/backup" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}