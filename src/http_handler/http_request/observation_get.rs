use super::request_common::{HTTPRequest, HTTPRequestType};
use super::observation::ObservationResponse;

#[derive(serde::Serialize)]
pub struct ObservationRequest{}

impl Into<HTTPRequest<Self>> for ObservationRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Get(self)
    }
}

impl HTTPRequestType for ObservationRequest {
    type Response = ObservationResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/observation" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}