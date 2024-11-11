pub(crate) enum HTTPRequest<T>
where
    T: HTTPRequestType,
{
    Get(T),
    Post(T),
    Put(T),
    Delete(T),
}

pub(crate) trait HTTPRequestType : Into<HTTPRequest<Self>>
{
    type Response: for<'de> serde::Deserialize<'de>;
    type Body: serde::Serialize;
    fn endpoint(&self) -> &str;
    fn body(&self) -> &Self::Body;

    fn header_params(&self) -> reqwest::header::HeaderMap;
}

