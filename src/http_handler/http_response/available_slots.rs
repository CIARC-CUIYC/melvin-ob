use crate::http_handler::http_handler_common::CommunicationSlot;
use crate::http_handler::http_response::response_common::{HTTPResponseType, JSONBodyHTTPResponseType, ResponseError};

#[derive(serde::Deserialize, Debug)]
pub struct AvailableSlotsResponse {
    communication_slots_used: usize,
    slots: Vec<CommunicationSlot>
}

impl JSONBodyHTTPResponseType for AvailableSlotsResponse {}

impl HTTPResponseType for AvailableSlotsResponse {
    type ParsedResponseType = Self;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Ok(Self::parse_json_body(response).await?)
    }
}