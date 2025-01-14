use super::response_common::{HTTPResponseType, ResponseError};
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::HTTPError;
use std::collections::HashMap;
use std::fmt::{Debug, Display};
use std::io::ErrorKind;
use strum_macros::Display;

#[derive(Debug)]
pub(crate) enum HTTPRequestMethod {
    Get,
    Post,
    Put,
    Delete,
}

#[derive(Debug, Display)]
pub enum RequestError {
    NotFound,
    FailedToOpen,
    Unknown,
}

impl std::error::Error for RequestError {}

impl From<std::io::Error> for RequestError {
    fn from(value: std::io::Error) -> Self {
        match value.kind() {
            ErrorKind::NotFound => RequestError::NotFound,
            ErrorKind::PermissionDenied => RequestError::FailedToOpen,
            _ => RequestError::Unknown,
        }
    }
}

pub(crate) trait JSONBodyHTTPRequestType: HTTPRequestType {
    type Body: serde::Serialize;
    fn body(&self) -> &Self::Body;
    fn header_params_with_content_type(&self) -> reqwest::header::HeaderMap {
        let mut headers = self.header_params();
        headers.append(
            "Content-Type",
            reqwest::header::HeaderValue::from_static("application/json"),
        );
        headers
    }
    async fn send_request(
        &self,
        client: &HTTPClient,
    ) -> Result<<Self::Response as HTTPResponseType>::ParsedResponseType, HTTPError> {
        let response = self
            .get_request_base(client)
            .headers(self.header_params_with_content_type())
            .query(&self.query_params())
            .json(&self.body())
            .send()
            .await;
        let response = response.map_err(ResponseError::from);
        Self::Response::read_response(response.map_err(HTTPError::HTTPResponseError)?)
            .await
            .map_err(HTTPError::HTTPResponseError)
    }
}

pub(crate) trait NoBodyHTTPRequestType: HTTPRequestType {
    async fn send_request(
        &self,
        client: &HTTPClient,
    ) -> Result<<Self::Response as HTTPResponseType>::ParsedResponseType, HTTPError> {
        let response = self
            .get_request_base(client)
            .headers(self.header_params())
            .query(&self.query_params())
            .send()
            .await;
        let response = response.map_err(ResponseError::from);
        Self::Response::read_response(response.map_err(HTTPError::HTTPResponseError)?)
            .await
            .map_err(HTTPError::HTTPResponseError)
    }
}

pub(crate) trait MultipartBodyHTTPRequestType: HTTPRequestType {
    async fn body(&self) -> Result<reqwest::multipart::Form, RequestError>;
    fn header_params_with_content_type(&self) -> reqwest::header::HeaderMap {
        let mut headers = self.header_params();
        headers.append(
            "Content-Type",
            reqwest::header::HeaderValue::from_static("multipart/form-data"),
        );
        headers
    }
    async fn send_request(
        &self,
        client: &HTTPClient,
    ) -> Result<<Self::Response as HTTPResponseType>::ParsedResponseType, HTTPError> {
        let response = self
            .get_request_base(client)
            .headers(self.header_params_with_content_type())
            .query(&self.query_params())
            .multipart(
                self.body()
                    .await
                    .map_err(HTTPError::HTTPRequestError)?,
            )
            .send()
            .await;
        let response = response.map_err(ResponseError::from);
        Self::Response::read_response(response.map_err(HTTPError::HTTPResponseError)?)
            .await
            .map_err(HTTPError::HTTPResponseError)
    }
}

pub(crate) trait HTTPRequestType {
    type Response: HTTPResponseType;
    fn endpoint(&self) -> &str;
    fn request_method(&self) -> HTTPRequestMethod;
    fn header_params(&self) -> reqwest::header::HeaderMap { reqwest::header::HeaderMap::default() }
    fn query_params(&self) -> HashMap<&str, String> { HashMap::new() }
    fn get_request_base(&self, client: &HTTPClient) -> reqwest::RequestBuilder {
        let compound_url = format!("{}{}", client.url(), self.endpoint());
        match self.request_method() {
            HTTPRequestMethod::Get => client.client().get(compound_url),
            HTTPRequestMethod::Post => client.client().post(compound_url),
            HTTPRequestMethod::Put => client.client().put(compound_url),
            HTTPRequestMethod::Delete => client.client().delete(compound_url),
        }
    }
}

pub fn bool_to_string(value: bool) -> &'static str {
    if value {
        "true"
    } else {
        "false"
    }
}
