use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::available_slots::AvailableSlotsResponse;

#[derive(Debug)]
pub struct AvailableSlotsRequest {}

impl NoBodyHTTPRequestType for AvailableSlotsRequest {}

impl HTTPRequestType for AvailableSlotsRequest {
    type Response = AvailableSlotsResponse;
    fn endpoint(&self) -> &str { "/slots" }
    fn request_method(&self) -> HTTPRequestMethod {HTTPRequestMethod::Get}
    
}