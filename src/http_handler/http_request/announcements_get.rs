use super::request_common::{HTTPRequest, HTTPRequestType};
use super::annoucements::AnnouncementsResponse ;

#[derive(serde::Serialize)]
pub struct AnnouncementsRequest{}

impl Into<HTTPRequest<Self>> for AnnouncementsRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Get(self)
    }
}

impl HTTPRequestType for AnnouncementsRequest {
    type Response = AnnouncementsResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/annoucements" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}