use crate::http_handler::{http_response::response_common::SerdeJSONBodyHTTPResponseType, common::CommunicationSlot};

#[derive(serde::Deserialize, Debug)]
pub struct AvailableSlotsResponse {
    communication_slots_used: usize,
    slots: Vec<CommunicationSlot>,
}

impl SerdeJSONBodyHTTPResponseType for AvailableSlotsResponse {}
