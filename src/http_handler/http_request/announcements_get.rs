use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::annoucements::AnnouncementsResponse;

#[derive(Debug)]
pub struct AnnouncementsRequest {}

impl NoBodyHTTPRequestType for AnnouncementsRequest {}

impl HTTPRequestType for AnnouncementsRequest {
    type Response = AnnouncementsResponse;
    fn endpoint(&self) -> &str { "/annoucements" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}