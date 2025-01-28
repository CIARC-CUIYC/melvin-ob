use super::objective_image::ObjectiveImageResponse;
use super::request_common::{
    HTTPRequestMethod, HTTPRequestType, MultipartBodyHTTPRequestType, RequestError,
};
use bytes::Bytes;
use std::collections::HashMap;

#[derive(Debug)]
pub struct ObjectiveImageRequest {
    encoded_image: Bytes,
    objective_id: usize,
}

impl MultipartBodyHTTPRequestType for ObjectiveImageRequest {
    async fn body(&self) -> Result<reqwest::multipart::Form, RequestError> {
        let file_part = reqwest::multipart::Part::stream(self.encoded_image.clone());
        Ok(reqwest::multipart::Form::new().part("image", file_part))
    }
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
    pub fn new(objective_id: usize, encoded_image: Vec<u8>) -> Self {
        Self {
            encoded_image: Bytes::from(encoded_image),
            objective_id,
        }
    }
}
