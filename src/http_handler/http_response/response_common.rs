use strum_macros::Display;

pub(crate) trait JSONBodyHTTPResponseType: HTTPResponseType {
    async fn parse_json_body(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError>
    where Self::ParsedResponseType: for<'de> serde::Deserialize<'de> {
        Ok(response.json::<Self::ParsedResponseType>().await?)
    }
}

pub(crate) trait SerdeJSONBodyHTTPResponseType {}

impl<T> JSONBodyHTTPResponseType for T
where
    T: SerdeJSONBodyHTTPResponseType,
    for<'de> T: serde::Deserialize<'de>,
{
}

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

pub(crate) trait ByteStreamResponseType: HTTPResponseType {}

pub(crate) trait HTTPResponseType {
    type ParsedResponseType;
    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError>;

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

#[derive(Debug, serde::Deserialize)]
pub struct BadRequestReturn {
    detail: Vec<BadRequestDetail>,
}

#[derive(Debug, serde::Deserialize)]
struct BadRequestDetail {
    error_type: String,
    loc: Vec<String>,
    msg: String,
    input: Option<String>,
    ctx: Option<BadRequestDetailContext>,
}

#[derive(Debug, serde::Deserialize)]
struct BadRequestDetailContext {
    expected: String,
}

#[derive(Debug, Display)]
pub enum ResponseError {
    InternalServer,
    BadRequest(BadRequestReturn),
    NoConnection,
    Unknown,
}

impl std::error::Error for ResponseError {}
impl From<reqwest::Error> for ResponseError {
    fn from(value: reqwest::Error) -> Self {
        if value.is_request() {
            ResponseError::BadRequest(BadRequestReturn { detail: vec![] })
        } else if value.is_timeout() || value.is_redirect() {
            ResponseError::InternalServer
        } else if value.is_connect() {
            ResponseError::NoConnection
        } else {
            ResponseError::Unknown
        }
    }
}
