use super::request_common::{bool_to_header_value, HTTPRequest, HTTPRequestType};
use super::modify_slot::ModifySlotResponse;


#[derive(serde::Serialize, Debug)]
pub struct ModifySlotRequest {
    pub slot_id: usize,
    pub enabled: bool,
}

impl Into<HTTPRequest<Self>> for ModifySlotRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Put(self)
    }
}

impl HTTPRequestType for ModifySlotRequest {
    type Response = ModifySlotResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/slots" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.append("slot_id", reqwest::header::HeaderValue::from(self.slot_id));
        headers.append("enabled", bool_to_header_value(self.enabled));
        headers
    }
}