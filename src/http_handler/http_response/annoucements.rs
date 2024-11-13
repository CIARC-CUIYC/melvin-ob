use crate::http_handler::http_response::response_common::{HTTPResponseType,
                                                          ResponseError, ByteStreamResponseType};

pub struct AnnouncementsResponse {}

impl ByteStreamResponseType for AnnouncementsResponse {}

impl HTTPResponseType for AnnouncementsResponse {
    type ParsedResponseType =
    std::pin::Pin<Box<dyn futures_core::Stream<Item=reqwest::Result<bytes::Bytes>> + Send>>;

    async fn read_response(response: reqwest::Response)
                           -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response)?;
        Ok(Box::pin(response.bytes_stream()))
    }
}