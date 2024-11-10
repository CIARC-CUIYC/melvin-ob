use crate::http_handler::serde;
use crate::http_handler::chrono;

// TODO: 422 Response Code: Validation Error -> not implemented

#[derive(serde::Deserialize)]
pub struct ModifySlotResponse{
    id: usize,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    enabled: bool
}

