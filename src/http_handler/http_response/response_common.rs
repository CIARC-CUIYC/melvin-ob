use strum_macros::Display;

/// Trait representing types that define how to parse HTTP responses.
pub(crate) trait HTTPResponseType {
    /// The resulting type returned after parsing the HTTP response body.
    type ParsedResponseType;

    /// Reads and parses the response body.
    ///
    /// # Arguments
    /// * `response` – A `reqwest::Response` object received from the HTTP client.
    ///
    /// # Returns
    /// * `Result<Self::ParsedResponseType, ResponseError>` – Parsed value or error.
    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError>;

    /// Unwraps and checks the HTTP status code.
    ///
    /// Returns the original response if the status is a success.
    /// Otherwise, returns an appropriate [`ResponseError`] based on the code.
    ///
    /// # Arguments
    /// * `response` – A `reqwest::Response` object.
    ///
    /// # Returns
    /// * `Ok(response)` if status is 2xx.
    /// * `Err(ResponseError)` if status is 4xx/5xx
    async fn unwrap_return_code(
        response: reqwest::Response,
    ) -> Result<reqwest::Response, ResponseError> {
        if response.status().is_success() {
            Ok(response)
        } else if response.status().is_server_error() {
            Err(ResponseError::InternalServer)
        } else if response.status().is_client_error() {
            Err(ResponseError::BadRequest(response.json().await?))
        } else {
            Err(ResponseError::Unknown)
        }
    }
}

/// Extension trait for response types that can be deserialized from JSON.
///
/// This trait provides a default `parse_json_body` implementation.
pub(crate) trait JSONBodyHTTPResponseType: HTTPResponseType {
    /// Parses a JSON response body into the target type.
    ///
    /// # Type Bounds
    /// * `Self::ParsedResponseType`: must implement `serde::Deserialize`.
    ///
    /// # Arguments
    /// * `response` – HTTP response object.
    ///
    /// # Returns
    /// * `Result<Self::ParsedResponseType, ResponseError>` – Parsed deserialized response.
    async fn parse_json_body(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError>
    where Self::ParsedResponseType: for<'de> serde::Deserialize<'de> {
        Ok(response.json::<Self::ParsedResponseType>().await?)
    }
}

impl<T> JSONBodyHTTPResponseType for T
where
    T: SerdeJSONBodyHTTPResponseType,
    for<'de> T: serde::Deserialize<'de>,
{
}

/// Marker trait for types that expect JSON as an HTTP response body and can be deserialized.
///
/// Implementors must also implement `serde::Deserialize`.
pub(crate) trait SerdeJSONBodyHTTPResponseType {}

impl<T> HTTPResponseType for T
where
    T: SerdeJSONBodyHTTPResponseType,
    for<'de> T: serde::Deserialize<'de>,
{
    type ParsedResponseType = T;

    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        Self::parse_json_body(resp).await
    }
}

/// Marker trait for types whose HTTP responses are raw byte streams
/// instead of JSON or structured data.
pub(crate) trait ByteStreamResponseType: HTTPResponseType {}

/// Top-level error type for handling all HTTP response-related failures.
#[derive(Debug, Display)]
pub enum ResponseError {
    /// A server-side error (HTTP 5xx or timeout).
    InternalServer,
    /// A client-side error (HTTP 4xx), parsed into a structured response.
    BadRequest(BadRequestReturn),
    /// A connection could not be established.
    NoConnection,
    /// Any other unexpected or unclassified error.
    Unknown,
}

impl std::error::Error for ResponseError {}
impl From<reqwest::Error> for ResponseError {
    /// Converts a `reqwest::Error` into a more specific `ResponseError` variant.
    fn from(value: reqwest::Error) -> Self {
        if value.is_request() {
            ResponseError::BadRequest(BadRequestReturn { detail: value.to_string() })
        } else if value.is_timeout() || value.is_redirect() {
            ResponseError::InternalServer
        } else if value.is_connect() {
            ResponseError::NoConnection
        } else {
            ResponseError::Unknown
        }
    }
}

/// Error detail returned when the backend responds with a client error (HTTP 4xx).
#[derive(Debug, serde::Deserialize)]
pub(crate) struct BadRequestReturn {
    /// Human-readable error explanation.
    detail: String,
}

/// Low-level error structure containing granular details about the failed request.
///
/// Usually used internally in `BadRequestReturn`.
#[derive(Debug, serde::Deserialize)]
struct BadRequestDetail {
    /// Type of validation or decoding error.
    error_type: String,
    /// Location of the error in the request body.
    loc: Vec<String>,
    /// Human-readable error message.
    msg: String,
    /// Input value that failed validation (if applicable).
    input: Option<String>,
    /// Additional error context provided by the backend.
    ctx: Option<BadRequestDetailContext>,
}

/// Additional context information for decoding/parsing failures.
#[derive(Debug, serde::Deserialize)]
struct BadRequestDetailContext {
    /// Expected type or format of the input value.
    expected: String,
}
