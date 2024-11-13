use crate::http_handler::HTTPError;

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

    fn unwrap_return_code(response: reqwest::Response) -> Result<reqwest::Response, ResponseError> {
        if response.status().is_success() {
            Ok(response)
        } else if response.status().is_server_error() {
            Err(ResponseError::InternalServerError)
        } else if response.status().is_client_error() {
            Err(ResponseError::BadRequest( BadRequestReturn{}))
        } else {
            Err(ResponseError::UnknownError)
        }
    }
}

pub struct BadRequestReturn{
    
}

pub enum ResponseError {
    InternalServerError,
    BadRequest(BadRequestReturn),
    UnknownError,
}

// TODO: split up Error types -> general enum HTTPError with possible Request or Response Error
impl From<std::io::Error> for ResponseError {
    fn from(value: std::io::Error) -> Self {
        ResponseError::UnknownError
    }
}

impl From<reqwest::Error> for ResponseError {
    fn from(value: reqwest::Error) -> Self {
        ResponseError::UnknownError
    }
}