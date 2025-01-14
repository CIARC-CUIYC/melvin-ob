use super::request_common::{HTTPRequestMethod, HTTPRequestType, NoBodyHTTPRequestType};
use super::shoot_image::ShootImageResponse;

#[derive(Debug)]
pub struct ShootImageRequest {}

impl NoBodyHTTPRequestType for ShootImageRequest {}

impl HTTPRequestType for ShootImageRequest {
    type Response = ShootImageResponse;
    fn endpoint(&self) -> &str {
        "/image"
    }

    fn request_method(&self) -> HTTPRequestMethod {
        HTTPRequestMethod::Get
    }
}
