use std::io;
use std::path::Path;
use super::daily_map::DailyMapResponse;
use super::request_common::{HTTPRequestMethod, HTTPRequestType,
                            MultipartBodyHTTPRequestType, RequestError};


#[derive(Debug)]
pub struct DailyMapRequest {
    image_path: String,
}

impl MultipartBodyHTTPRequestType for DailyMapRequest {
    async fn body(&self) -> Result<reqwest::multipart::Form, RequestError> {
        let file_part = reqwest::multipart::Part::file(&self.image_path).await?;
        Ok(reqwest::multipart::Form::new().part("image", file_part))
    }
}

impl HTTPRequestType for DailyMapRequest {
    type Response = DailyMapResponse;
    fn endpoint(&self) -> &str { "/dailyMap" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Post }
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