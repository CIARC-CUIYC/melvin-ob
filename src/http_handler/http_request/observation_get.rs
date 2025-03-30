use super::observation::ObservationResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

/// Request type for the /observation endpoint -> GET.
#[derive(Debug)]
pub(crate) struct ObservationRequest {}

impl NoBodyHTTPRequestType for ObservationRequest {}

impl HTTPRequestType for ObservationRequest {
    /// Type of the expected response.
    type Response = ObservationResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/observation" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
