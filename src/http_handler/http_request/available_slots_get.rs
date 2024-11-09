use super::request_common::HTTPRequestType;
use super::available_slots::AvailableSlotsResponse;

struct AvailableSlotsRequest {}

impl HTTPRequestType for AvailableSlotsRequest {
    type Response = AvailableSlotsResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/slots" }
    fn body(&self) -> &Self::Body { &() }
}