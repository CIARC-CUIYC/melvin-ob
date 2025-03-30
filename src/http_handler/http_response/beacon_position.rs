use crate::http_handler::http_response::response_common::SerdeJSONBodyHTTPResponseType;

/// Response type for the /beacon endpoint
#[derive(serde::Deserialize, Debug)]
pub(crate) struct BeaconPositionResponse {
    /// Status of the Beacon
    status: String,
    /// Already made attempts for guessing the position of this beacon
    attempts_made: i8,
}

impl BeaconPositionResponse {
    /// `true` if the message indicates that the beacon was correctly located
    pub(crate) fn is_success(&self) -> bool { self.status.contains("The beacon was found!") }
    /// Returns the number of guesses that were already submitted
    pub(crate) fn attempts_made(&self) -> i8 { self.attempts_made }
    /// `true` if the beacon objective is not known to the server
    pub(crate) fn is_unknown(&self) -> bool { self.status.contains("Could not find beacon") }
    /// `true` if the beacon could not be correctly located
    pub(crate) fn is_fail(&self) -> bool { self.status.contains("The beacon could not be found") }
    /// `true` if the beacon could not be correctly located and this was the last possible guess
    pub(crate) fn is_last(&self) -> bool { self.status.contains("No more rescue attempts") }
    /// Returns the raw status message
    pub(crate) fn msg(&self) -> &str { &self.status }
}

impl SerdeJSONBodyHTTPResponseType for BeaconPositionResponse {}
