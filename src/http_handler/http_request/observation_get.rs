use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::observation::ObservationResponse;

#[derive(Debug)]
pub struct ObservationRequest {}

impl NoBodyHTTPRequestType for ObservationRequest {}

impl HTTPRequestType for ObservationRequest {
    type Response = ObservationResponse;
    fn endpoint(&self) -> &str { "/observation" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}