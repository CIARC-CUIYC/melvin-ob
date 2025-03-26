use super::objective_image::ObjectiveImageResponse;
use super::request_common::{
    HTTPRequestMethod, HTTPRequestType, MultipartBodyHTTPRequestType,
};
use std::collections::HashMap;
use std::path::PathBuf;

#[derive(Debug)]
pub struct ObjectiveImageRequest {
    image_path: PathBuf,
    objective_id: usize,
}

impl MultipartBodyHTTPRequestType for ObjectiveImageRequest {
    fn image_path(&self) -> &PathBuf { &self.image_path }
}

impl HTTPRequestType for ObjectiveImageRequest {
    type Response = ObjectiveImageResponse;
    fn endpoint(&self) -> &'static str { "/image" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Post }
    fn query_params(&self) -> HashMap<&str, String> {
        let mut query = HashMap::new();
        query.insert("objective_id", self.objective_id.to_string());
        query
    }
}

impl ObjectiveImageRequest {
    pub fn new(objective_id: usize, image_path: PathBuf) -> Self {
        Self { image_path, objective_id }
    }
}
