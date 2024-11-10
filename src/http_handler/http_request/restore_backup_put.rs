use serde_with::serde_derive::Serialize;
use super::request_common::HTTPRequestType;
use super::restore_backup::RestoreBackupResponse;

#[derive(Serialize)]
struct RestoreBackupRequest {}

impl HTTPRequestType for RestoreBackupRequest {
    type Response = RestoreBackupResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/backup"}
    fn body(&self) -> &Self::Body {&()}
}