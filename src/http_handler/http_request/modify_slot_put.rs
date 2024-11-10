use super::request_common::HTTPRequestType;
use super::modify_slot::ModifySlotResponse;

#[derive(serde::Serialize)]
struct ModifySlotRequest {
    slot_id: usize,
    enabled: bool,
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