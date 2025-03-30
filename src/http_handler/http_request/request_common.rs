use super::response_common::{HTTPResponseType, ResponseError};
use crate::http_handler::{HTTPError, http_client::HTTPClient};
use std::{fmt::Debug, io::ErrorKind};
use std::collections::HashMap;
use std::path::PathBuf;
use strum_macros::Display;

/// Base trait for all types representing HTTP requests.
///
/// Each implementor must define the associated response type and required metadata
/// such as endpoint, HTTP method, headers, and query parameters.
pub(crate) trait HTTPRequestType {
    /// The type of the expected HTTP response.
    type Response: HTTPResponseType;

    /// Returns the request endpoint path, relative to the client's base URL.
    fn endpoint(&self) -> &str;

    /// Specifies the HTTP request method (GET, POST, etc.).
    fn request_method(&self) -> HTTPRequestMethod;

    /// Provides custom request headers. Defaults to an empty map.
    fn header_params(&self) -> reqwest::header::HeaderMap {
        reqwest::header::HeaderMap::default()
    }

    /// Provides URL query parameters. Defaults to an empty map.
    fn query_params(&self) -> HashMap<&str, String> {
        HashMap::new()
    }

    /// Creates the base `RequestBuilder` from the HTTP client, applying method and URL.
    ///
    /// # Arguments
    /// * `client` – The preconfigured `HTTPClient` used to perform the request.
    ///
    /// # Returns
    /// * A `reqwest::RequestBuilder` ready for customization (headers, body, etc.).
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


/// Enum representing the four primary HTTP request methods.
#[derive(Debug)]
pub(crate) enum HTTPRequestMethod {
    /// HTTP GET request.
    Get,
    /// HTTP POST request.
    Post,
    /// HTTP PUT request.
    Put,
    /// HTTP DELETE request.
    Delete,
}


/// Errors that may occur while constructing or performing an HTTP request.
#[derive(Debug, Display)]
pub(crate) enum RequestError {
    /// The requested file or resource could not be found.
    NotFound,
    /// A file or resource could not be opened due to permissions.
    FailedToOpen,
    /// An unknown or unspecified request error occurred.
    Unknown,
}

impl std::error::Error for RequestError {}

impl From<std::io::Error> for RequestError {
    /// Maps standard I/O errors to [`RequestError`] variants.
    fn from(value: std::io::Error) -> Self {
        match value.kind() {
            ErrorKind::NotFound => RequestError::NotFound,
            ErrorKind::PermissionDenied => RequestError::FailedToOpen,
            _ => RequestError::Unknown,
        }
    }
}

/// Trait for request types that send a JSON body and expect a structured response.
///
/// Requires a `Body` type implementing `serde::Serialize`.
pub(crate) trait JSONBodyHTTPRequestType: HTTPRequestType {
    /// Type of the serializable request body.
    type Body: serde::Serialize;

    /// Returns a reference to the request body.
    fn body(&self) -> &Self::Body;

    /// Constructs a header map with `"Content-Type: application/json"` pre-filled.
    fn header_params_with_content_type(&self) -> reqwest::header::HeaderMap {
        let mut headers = self.header_params();
        headers.append(
            "Content-Type",
            reqwest::header::HeaderValue::from_static("application/json"),
        );
        headers
    }

    /// Sends the request with a JSON-encoded body.
    ///
    /// # Arguments
    /// * `client` – The shared HTTP client instance.
    ///
    /// # Returns
    /// * Parsed response value or an `HTTPError`.
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
        let resp = response.map_err(ResponseError::from);
        Self::Response::read_response(resp.map_err(HTTPError::HTTPResponseError)?)
            .await
            .map_err(HTTPError::HTTPResponseError)
    }
}


/// Trait for requests that do not include a request body.
pub(crate) trait NoBodyHTTPRequestType: HTTPRequestType {
    /// Sends a request with no body.
    ///
    /// # Arguments
    /// * `client` – The HTTP client instance.
    ///
    /// # Returns
    /// * Parsed response value or an `HTTPError`.
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
        let resp = response.map_err(ResponseError::from);
        Self::Response::read_response(resp.map_err(HTTPError::HTTPResponseError)?)
            .await
            .map_err(HTTPError::HTTPResponseError)
    }
}

/// Trait for requests that send multipart form data (e.g., file uploads).
///
/// Requires a file path to construct a `multipart/form-data` body.
pub(crate) trait MultipartBodyHTTPRequestType: HTTPRequestType {
    /// Assembles the multipart form body from the image path.
    ///
    /// # Returns
    /// * A multipart form with the image file attached.
    async fn body(&self) -> Result<reqwest::multipart::Form, RequestError> {
        let file_part = reqwest::multipart::Part::file(self.image_path()).await?;
        Ok(reqwest::multipart::Form::new().part("image", file_part))
    }

    /// Returns the absolute or relative path to the image file.
    fn image_path(&self) -> &PathBuf;

    /// Sends the multipart form request.
    ///
    /// # Arguments
    /// * `client` – The HTTP client instance.
    ///
    /// # Returns
    /// * Parsed response value or an `HTTPError`.
    async fn send_request(
        &self,
        client: &HTTPClient,
    ) -> Result<<Self::Response as HTTPResponseType>::ParsedResponseType, HTTPError> {
        let response = self
            .get_request_base(client)
            .headers(self.header_params())
            .query(&self.query_params())
            .multipart(self.body().await.map_err(HTTPError::HTTPRequestError)?)
            .send()
            .await;
        let resp = response.map_err(ResponseError::from);
        Self::Response::read_response(resp.map_err(HTTPError::HTTPResponseError)?)
            .await
            .map_err(HTTPError::HTTPResponseError)
    }
}

/// Converts a `bool` value to a string slice (`"true"` or `"false"`).
///
/// Useful for generating query parameters.
///
/// # Arguments
/// * `value` – Boolean value to stringify.
///
/// # Returns
/// * `"true"` or `"false"` as `&'static str`.
pub(super) fn bool_to_string(value: bool) -> &'static str {
    if value { "true" } else { "false" }
}
