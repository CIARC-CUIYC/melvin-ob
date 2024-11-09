use crate::http_handler::serde;
use crate::http_handler::chrono;

#[derive(serde::Deserialize)]
pub struct AvailableSlotsResponse{
    communication_slots_used: usize,
    slots: Vec<CommunicationSlot>,
}

#[derive(serde::Deserialize)]
struct CommunicationSlot{
    id: usize,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    enabled: bool
}