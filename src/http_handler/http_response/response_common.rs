pub(crate) trait JSONBodyHTTPResponseType: HTTPResponseType {
    async fn parse_json_body(response: reqwest::Response)
                             -> Result<Self::ParsedResponseType, ResponseError>
    where
        Self::ParsedResponseType: for<'de> serde::Deserialize<'de>,
    { Ok(response.json::<Self::ParsedResponseType>().await?) }
}

pub(crate) trait ByteStreamResponseType: HTTPResponseType {}


pub(crate) trait HTTPResponseType {
    type ParsedResponseType;
    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError>;

    async fn unwrap_return_code(response: reqwest::Response) -> Result<reqwest::Response, ResponseError> {
        if response.status().is_success() {
            Ok(response)
        } else if response.status().is_server_error() {
            Err(ResponseError::InternalServerError)
        } else if response.status().is_client_error() {
            Err(ResponseError::BadRequest(response.json().await?))
        } else {
            Err(ResponseError::UnknownError)
        }
    }
}

#[derive(Debug, serde::Deserialize)]
struct BadRequestReturn {
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

pub enum ResponseError {
    InternalServerError,
    BadRequest(BadRequestReturn),
    NoConnectionError,
    UnknownError,
}

impl From<reqwest::Error> for ResponseError {
    fn from(value: reqwest::Error) -> Self {
        if value.is_request() {
            ResponseError::BadRequest(BadRequestReturn { detail: vec![] })
        } else if value.is_timeout() || value.is_redirect() {
            ResponseError::InternalServerError
        } else if value.is_connect() {
            ResponseError::NoConnectionError
        } else {
            ResponseError::UnknownError
        }
    }
}