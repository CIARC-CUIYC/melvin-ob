use super::annoucements::AnnouncementsResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

/// Request type for the /announcements endpoint.
#[derive(Debug)]
pub struct AnnouncementsRequest {}

impl NoBodyHTTPRequestType for AnnouncementsRequest {}

impl HTTPRequestType for AnnouncementsRequest {
    /// Type of the expected response.
    type Response = AnnouncementsResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/annoucements" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
