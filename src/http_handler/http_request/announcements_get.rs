use super::annoucements::AnnouncementsResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

#[derive(Debug)]
pub struct AnnouncementsRequest {}

impl NoBodyHTTPRequestType for AnnouncementsRequest {}

impl HTTPRequestType for AnnouncementsRequest {
    type Response = AnnouncementsResponse;
    fn endpoint(&self) -> &'static str {
        "/annoucements"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Get
    }
}
