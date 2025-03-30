use super::modify_slot::ModifySlotResponse;
use super::request_common::{
    HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType, bool_to_string,
};
use std::collections::HashMap;

/// Request type for the /slots endpoint -> PUT.
#[derive(Debug)]
pub(crate) struct ModifySlotRequest {
    /// The id of the slot.
    pub(crate) slot_id: usize,
    /// The status of the slot.
    pub(crate) enabled: bool,
}

impl NoBodyHTTPRequestType for ModifySlotRequest {}

impl HTTPRequestType for ModifySlotRequest {
    /// Type of the expected response.
    type Response = ModifySlotResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/slots" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
    /// A `HashMap` containing the query param key value pairs.
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert("slot_id", self.slot_id.to_string());
        query.insert("enabled", bool_to_string(self.enabled).to_string());
        query
    }
}
