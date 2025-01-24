use std::collections::HashMap;
use super::modify_slot::ModifySlotResponse;
use super::request_common::{
    bool_to_string, HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType,
};

#[derive(Debug)]
pub struct ModifySlotRequest {
    pub slot_id: usize,
    pub enabled: bool,
}

impl NoBodyHTTPRequestType for ModifySlotRequest {}

impl HTTPRequestType for ModifySlotRequest {
    type Response = ModifySlotResponse;
    fn endpoint(&self) -> &'static str {
        "/slots"
    }
    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Put
    }
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert("slot_id", self.slot_id.to_string());
        query.insert("enabled", bool_to_string(self.enabled).to_string());
        query
    }
}
