use crate::http_handler::http_response::response_common::{HTTPResponseType, JSONBodyHTTPResponseType, ResponseError, SerdeJSONBodyHTTPResponseType};

#[derive(serde::Deserialize, Debug)]
pub struct ModifySlotResponse {
    id: usize,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    enabled: bool,
}

impl SerdeJSONBodyHTTPResponseType for ModifySlotResponse {}