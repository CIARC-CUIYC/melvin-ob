use crate::http_handler::serde;
use crate::http_handler::http_handler_common::CommunicationSlot;

#[derive(serde::Deserialize)]
pub struct AvailableSlotsResponse{
    communication_slots_used: usize,
    slots: Vec<CommunicationSlot>,
}