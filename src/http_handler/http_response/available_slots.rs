use crate::http_handler::{
    common::CommunicationSlot, http_response::response_common::SerdeJSONBodyHTTPResponseType,
};

#[derive(serde::Deserialize, Debug)]
pub struct AvailableSlotsResponse {
    communication_slots_used: usize,
    slots: Vec<CommunicationSlot>,
}

impl SerdeJSONBodyHTTPResponseType for AvailableSlotsResponse {}
