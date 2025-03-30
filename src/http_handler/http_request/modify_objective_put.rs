use super::modify_objective;
use super::request_common::{HTTPRequestMethod, HTTPRequestType, JSONBodyHTTPRequestType};
use crate::http_handler::common::{BeaconObjective, ImageObjective};

/// Request type for the /objective endpoint -> PUT.
#[derive(serde::Serialize, Debug)]
#[cfg(debug_assertions)]
pub(crate) struct ModifyObjectiveRequest {
    /// `Vec` of changed/newly created `ImageObjective` objects.
    pub(crate) zoned_objectives: Vec<ImageObjective>,
    /// `Vec` of changed/newly created `BeaconObjective` objects.
    pub(crate) beacon_objectives: Vec<BeaconObjective>,
}

impl JSONBodyHTTPRequestType for ModifyObjectiveRequest {
    /// The type that is serializable into a json body.
    type Body = Self;
    /// Returns the body object to be serialized.
    fn body(&self) -> &Self::Body { self }
}

impl HTTPRequestType for ModifyObjectiveRequest {
    /// Type of the expected response.
    type Response = modify_objective::ModifyObjectiveResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/objective" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Put }
}
