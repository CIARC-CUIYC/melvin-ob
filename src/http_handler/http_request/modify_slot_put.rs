use serde_with::serde_derive::Serialize;
use super::request_common::HTTPRequestType;
use super::modify_slot::ModifySlotResponse;

#[derive(Serialize)]
struct ModifySlotRequest {
    slot_id: usize,
    enabled: bool
}

impl HTTPRequestType for ModifySlotRequest {
    type Response = ModifySlotResponse;
    type Body = ModifySlotRequest;
    fn endpoint(&self) -> &str { "/slots" }
    fn body(&self) -> &Self::Body {self}
}