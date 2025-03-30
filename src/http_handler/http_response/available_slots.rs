use crate::http_handler::{
    common::CommunicationSlot, http_response::response_common::SerdeJSONBodyHTTPResponseType,
};

/// Response type for the /slots endpoint -> GET
#[derive(serde::Deserialize, Debug)]
pub(crate) struct AvailableSlotsResponse {
    /// Number of already used communication slots
    communication_slots_used: usize,
    /// `Vec` of remaining `CommunicationSlot` objects
    slots: Vec<CommunicationSlot>,
}

impl SerdeJSONBodyHTTPResponseType for AvailableSlotsResponse {}
