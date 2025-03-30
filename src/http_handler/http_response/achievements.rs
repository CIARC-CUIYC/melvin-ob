use super::response_common::SerdeJSONBodyHTTPResponseType;
use crate::http_handler::common::Achievement;

/// Response type for the /achievements endpoint
#[derive(serde::Deserialize, Debug)]
pub(crate) struct AchievementsResponse {
    /// `Vec` of done `Achievement` objects 
    achievements: Vec<Achievement>,
}

impl SerdeJSONBodyHTTPResponseType for AchievementsResponse {}
