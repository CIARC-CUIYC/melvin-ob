use crate::http_handler::http_response::response_common::{
    HTTPResponseType, JSONBodyHTTPResponseType, ResponseError, SerdeJSONBodyHTTPResponseType,
};

// TODO: 422 Response Code: Validation Error -> not implemented

#[derive(serde::Deserialize, Debug)]
pub struct BeaconPositionResponse {
    status: String,
    attempts_made: i8,
}

impl SerdeJSONBodyHTTPResponseType for BeaconPositionResponse {}
