use super::request_common::{HTTPRequest, HTTPRequestType};
use super::modify_slot::ModifySlotResponse;

#[derive(serde::Serialize)]
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
        headers.append("slot_id", self.slot_id.into());
        headers.append("enabled", {
            if self.enabled {
                reqwest::header::HeaderValue::from_str("true").unwrap()
            } else {
                reqwest::header::HeaderValue::from_str("false").unwrap()
            }
        });
        headers
    }
}