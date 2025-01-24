use super::observation::ObservationResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

#[derive(Debug)]
pub struct ObservationRequest {}

impl NoBodyHTTPRequestType for ObservationRequest {}

impl HTTPRequestType for ObservationRequest {
    type Response = ObservationResponse;
    fn endpoint(&self) -> &'static str {
        "/observation"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Get
    }
}
