use std::io;
use std::path::Path;
use super::request_common::{HTTPRequest, HTTPRequestType};
use super::daily_map::DailyMapResponse;

#[derive(serde::Serialize)]
pub struct DailyMapRequest {
    image_path: String,
}

impl DailyMapRequest {
    pub fn new<P: AsRef<Path>>(image_path: P) -> Result<Self, io::Error> {
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
        })
    }
}

impl Into<HTTPRequest<Self>> for DailyMapRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Post(self)
    }
}

impl HTTPRequestType for DailyMapRequest {
    type Response = DailyMapResponse;

    type Body = ();

    fn endpoint(&self) -> &str { "/simulation" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        let mut headers = reqwest::header::HeaderMap::new();
        headers.insert("Content-Type",
                       reqwest::header::HeaderValue::from_static("multipart/form-data"));
        headers
    }
    
    async fn multipart_body(&self) -> Option<reqwest::multipart::Form> {
        let file_part = reqwest::multipart::Part::file(&self.image_path).await.unwrap();
        Some(reqwest::multipart::Form::new().part("image", file_part))
    }
}
