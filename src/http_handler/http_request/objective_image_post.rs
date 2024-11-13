use std::io;
use std::path::Path;
use super::objective_image::ObjectiveImageResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType,
                            MultipartBodyHTTPRequestType, RequestError};


#[derive(Debug)]
pub struct ObjectiveImageRequest {
    image_path: String,
    objective_id: usize,
}

impl MultipartBodyHTTPRequestType for ObjectiveImageRequest {
    async fn body(&self) -> Result<reqwest::multipart::Form, RequestError> {
        let file_part = reqwest::multipart::Part::file(&self.image_path).await?;
        Ok(reqwest::multipart::Form::new().part("image", file_part))
    }
}

impl HTTPRequestType for ObjectiveImageRequest {
    type Response = ObjectiveImageResponse;
    fn endpoint(&self) -> &str { "/image" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Post }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.insert("objective_id", reqwest::header::HeaderValue::from(self.objective_id));
        headers
    }
}

impl ObjectiveImageRequest {
    pub fn new<P: AsRef<Path>>(image_path: P, objective_id: usize) -> Result<Self, io::Error> {
        let path = image_path.as_ref();
        if !path.exists() {
            return Err(io::Error::new(io::ErrorKind::NotFound,
                                      "File path does not exist"));
        }
        if !path.is_file() {
            return Err(io::Error::new(io::ErrorKind::InvalidInput,
                                      "Path is not a valid file"));
        }
        Ok(Self {
            image_path: path.to_string_lossy().to_string(),
            objective_id,
        })
    }
}
