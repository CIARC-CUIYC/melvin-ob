use super::request_common::{HTTPRequest, HTTPRequestType};
use super::shoot_image::ShootImageResponse;

#[derive(serde::Serialize, Debug)]
pub struct ShootImageRequest{}

impl Into<HTTPRequest<Self>> for ShootImageRequest {
    fn into(self) -> HTTPRequest<Self> {
        HTTPRequest::Get(self)
    }
}

impl HTTPRequestType for ShootImageRequest {
    type Response = ShootImageResponse;
    type Body = ();
    fn endpoint(&self) -> &str { "/image" }
    fn body(&self) -> &Self::Body { &() }
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }
}