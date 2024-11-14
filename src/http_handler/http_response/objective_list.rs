use crate::http_handler::http_handler_common::{ZonedObjective, BeaconObjective};
use crate::http_handler::http_response::response_common::{HTTPResponseType,
                                                          JSONBodyHTTPResponseType, ResponseError};

#[derive(serde::Deserialize, Debug)]
pub struct ObjectiveListResponse {
    zoned_objectives: Vec<ZonedObjective>,
    beacon_objectives: Vec<BeaconObjective>,
}

impl JSONBodyHTTPResponseType for ObjectiveListResponse {}

impl HTTPResponseType for ObjectiveListResponse {
    type ParsedResponseType = Self;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Ok(Self::parse_json_body(response).await?)
    }
}