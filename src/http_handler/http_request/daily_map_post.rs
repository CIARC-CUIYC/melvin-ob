use super::daily_map::DailyMapResponse;
use super::request_common::{
    HTTPRequestMethod, HTTPRequestType, MultipartBodyHTTPRequestType,
};
use std::{io, path::Path};
use std::path::PathBuf;

/// Request type for the /dailyMap endpoint.
#[derive(Debug)]
pub(crate) struct DailyMapRequest {
    /// File path to the current daily map png image file.
    image_path: PathBuf,
}

impl MultipartBodyHTTPRequestType for DailyMapRequest {
    /// returns the path for the multipart image file.
    fn image_path(&self) -> &PathBuf { &self.image_path }
}

impl HTTPRequestType for DailyMapRequest {
    /// Type of the expected response.
    type Response = DailyMapResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/dailyMap" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Post }
}

impl DailyMapRequest {
    /// Constructs a new `DailyMapRequest` from an image path to the full snapshot.
    pub(crate) fn new<P: AsRef<Path>>(image_path: P) -> Result<Self, io::Error> {
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
