use super::request_common::{HTTPRequest, HTTPRequestType};
use super::available_slots::AvailableSlotsResponse;

#[derive(serde::Serialize)]
pub struct AvailableSlotsRequest {}

impl Into<HTTPRequest<Self>> for AvailableSlotsRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Get(self)
    }
}

impl HTTPRequestType for AvailableSlotsRequest {
    type Response = AvailableSlotsResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/slots" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}