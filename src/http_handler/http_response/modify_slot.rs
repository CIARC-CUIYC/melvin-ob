use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;
use chrono::{DateTime, Utc};

/// Response type for the /slots endpoint -> PUT
#[derive(serde::Deserialize, Debug)]
pub(crate) struct ModifySlotResponse {
    /// The id of the selected Communication Slot
    id: usize,
    /// The start time of the slot as a UTC Timestamp
    start: DateTime<Utc>,
    /// The end time of the slot as a UTC Timestamp
    end: DateTime<Utc>,
    /// Whether the slot is currently booked or not
    enabled: bool,
}

impl SerdeJSONBodyHTTPResponseType for ModifySlotResponse {}
