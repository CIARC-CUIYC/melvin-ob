use super::objective_image::ObjectiveImageResponse;
use super::request_common::{
    HTTPRequestMethod, HTTPRequestType, MultipartBodyHTTPRequestType,
};
use std::{collections::HashMap, path::PathBuf};

/// Request type for the /image endpoint -> POST.
#[derive(Debug)]
pub(crate) struct ObjectiveImageRequest {
    /// The path where the image png file is stored.
    image_path: PathBuf,
    /// The objective id.
    objective_id: usize,
}

impl MultipartBodyHTTPRequestType for ObjectiveImageRequest {
    /// returns the path to the multipart image png file.
    fn image_path(&self) -> &PathBuf { &self.image_path }
}

impl HTTPRequestType for ObjectiveImageRequest {
    /// Type of the expected response.
    type Response = ObjectiveImageResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/image" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Post }
    /// A `HashMap` containing the query param key value pairs
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert("objective_id", self.objective_id.to_string());
        query
    }
}

impl ObjectiveImageRequest {
    /// Creates a new `ObjectiveImageRequest` from an id and a png file path.
    pub fn new(objective_id: usize, image_path: PathBuf) -> Self {
        Self { image_path, objective_id }
    }
}
