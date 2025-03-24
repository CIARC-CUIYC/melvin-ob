use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;
use chrono::{DateTime, Utc};

#[derive(serde::Deserialize, Debug)]
pub struct ModifySlotResponse {
    id: usize,
    start: DateTime<Utc>,
    end: DateTime<Utc>,
    enabled: bool,
}

impl SerdeJSONBodyHTTPResponseType for ModifySlotResponse {}
