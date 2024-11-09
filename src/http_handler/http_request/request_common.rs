pub enum HTTPRequest<T>
where
    T: HTTPRequestType,
{
    Get(T),
    Post(T),
    Put(T),
    Delete(T),
}

pub trait HTTPRequestType {
    type Response: for<'de> serde::Deserialize<'de>;
    type Body: serde::Serialize;
    fn endpoint(&self) -> &str;
    fn body(&self) -> &Self::Body;
}

