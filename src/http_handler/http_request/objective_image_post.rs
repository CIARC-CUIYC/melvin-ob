use std::io;
use std::path::Path;
use super::request_common::{HTTPRequest, HTTPRequestType};
use super::objective_image::ObjectiveImageResponse;

#[derive(serde::Serialize, Debug)]
pub struct ObjectiveImageRequest {
    image_path: String,
    objective_id: usize
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
            objective_id
        })
    }
}

impl Into<HTTPRequest<Self>> for ObjectiveImageRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Post(self)
    }
}

impl HTTPRequestType for ObjectiveImageRequest {
    type Response = ObjectiveImageResponse;
    type Body = ();

    fn endpoint(&self) -> &str { "/image" }
    fn body(&self) -> &Self::Body { &() }
    
    async fn multipart_body(&self) -> Option<reqwest::multipart::Form> {
        let file_part = reqwest::multipart::Part::file(&self.image_path).await.unwrap();
        Some(reqwest::multipart::Form::new().part("image", file_part))
    }

    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.insert("Content-Type",
                       reqwest::header::HeaderValue::from_static("multipart/form-data"));
        headers.insert("objective_id", reqwest::header::HeaderValue::from(self.objective_id));
        headers
    }
}
