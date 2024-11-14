use reqwest::Response;
use crate::http_handler::http_handler_common::CommunicationSlot;
use crate::http_handler::http_response::response_common::{HTTPResponseType, JSONBodyHTTPResponseType, ResponseError, SerdeJSONBodyHTTPResponseType};

#[derive(serde::Deserialize, Debug)]
pub struct AvailableSlotsResponse {
    communication_slots_used: usize,
    slots: Vec<CommunicationSlot>
}

impl SerdeJSONBodyHTTPResponseType for AvailableSlotsResponse {}
