use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

// TODO: 422 Response Code: Validation Error -> not implemented

#[derive(serde::Deserialize, Debug)]
pub struct BeaconPositionResponse {
    status: String,
    attempts_made: i8,
}

impl BeaconPositionResponse {
    pub fn is_success(&self) -> bool {
        self.status.contains("The beacon was found!")
    }
    pub fn attempts_made(&self) -> i8 {
        self.attempts_made
    }
    pub fn is_unknown(&self) -> bool {
        self.status.contains("Could not find beacon")
    }
    pub fn is_fail(&self) -> bool {
        self.status.contains("The beacon could not be found")
    }
    pub fn is_last(&self) -> bool {
        self.status.contains("No more rescue attempts")
    }
    pub fn msg(&self) -> &str {
        &self.status
    }
}

impl SerdeJSONBodyHTTPResponseType for BeaconPositionResponse {}
