use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::shoot_image::ShootImageResponse;

/// Request type for the /image endpoint -> GET.
#[derive(Debug)]
pub struct ShootImageRequest {}

impl NoBodyHTTPRequestType for ShootImageRequest {}

impl HTTPRequestType for ShootImageRequest {
    /// Type of the expected response.
    type Response = ShootImageResponse;
    /// `str` object representing the specific endpoint.
    fn endpoint(&self) -> &'static str { "/image" }
    /// The corresponding HTTP Request Method.
    fn request_method(&self) -> HTTPRequestMethod { HTTPRequestMethod::Get }
}
