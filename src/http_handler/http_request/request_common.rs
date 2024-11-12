use reqwest::multipart;

pub(crate) enum HTTPRequest<T>
where
    T: HTTPRequestType,
{
    Get(T),
    Post(T),
    Put(T),
    Delete(T),
}

pub(crate) trait HTTPRequestType: Into<HTTPRequest<Self>>
{
    type Response: for<'de> serde::Deserialize<'de>;
    type Body: serde::Serialize;
    fn endpoint(&self) -> &str;
    fn body(&self) -> &Self::Body;
    async fn multipart_body(&self) -> Option<multipart::Form> {None}
    fn header_params(&self) -> reqwest::header::HeaderMap;
}

pub fn bool_to_header_value<'a>(value: bool) -> reqwest::header::HeaderValue {
    if value {
        reqwest::header::HeaderValue::from_str("true").unwrap()
    } else {
        reqwest::header::HeaderValue::from_str("false").unwrap()
    }
}

