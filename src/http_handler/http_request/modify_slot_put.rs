use super::modify_slot::ModifySlotResponse;
use super::request_common::{
    bool_to_header_value, HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType,
};

#[derive(Debug)]
pub struct ModifySlotRequest {
    pub slot_id: usize,
    pub enabled: bool,
}

impl NoBodyHTTPRequestType for ModifySlotRequest {}

impl HTTPRequestType for ModifySlotRequest {
    type Response = ModifySlotResponse;
    fn endpoint(&self) -> &str {
        "/slots"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Put
    }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.append("slot_id", reqwest::header::HeaderValue::from(self.slot_id));
        headers.append("enabled", bool_to_header_value(self.enabled));
        headers
    }
}
