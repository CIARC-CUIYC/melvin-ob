use super::daily_map::DailyMapResponse;
use super::request_common::{
    HTTPRequestMethod, HTTPRequestType, MultipartBodyHTTPRequestType,
};
use std::{io, path::Path};
use std::path::PathBuf;

#[derive(Debug)]
pub struct DailyMapRequest {
    image_path: PathBuf,
}

impl MultipartBodyHTTPRequestType for DailyMapRequest {
    fn image_path(&self) -> &PathBuf { &self.image_path }
}

impl HTTPRequestType for DailyMapRequest {
    type Response = DailyMapResponse;
    fn endpoint(&self) -> &'static str { "/dailyMap" }
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Post }
}

impl DailyMapRequest {
    pub fn new<P: AsRef<Path>>(image_path: P) -> Result<Self, io::Error> {
        let path = image_path.as_ref();
        if !path.exists() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                "File path does not exist",
            ));
        }
        if !path.is_file() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "Path is not a valid file",
            ));
        }
        Ok(Self { image_path: path.to_path_buf() })
    }
}
