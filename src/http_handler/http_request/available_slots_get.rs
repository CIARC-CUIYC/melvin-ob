use super::available_slots::AvailableSlotsResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};

/// Request type for the /slots endpoint -> GET.
#[derive(Debug)]
pub struct AvailableSlotsRequest {}

impl NoBodyHTTPRequestType for AvailableSlotsRequest {}

impl HTTPRequestType for AvailableSlotsRequest {
    /// Type of the expected response.
    type Response = AvailableSlotsResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/slots" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
