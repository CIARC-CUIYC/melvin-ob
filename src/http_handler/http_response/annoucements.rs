use crate::http_handler::http_response::response_common::{
    ByteStreamResponseType, HTTPResponseType, ResponseError,
};
use prost::bytes::Bytes;

pub struct AnnouncementsResponse {}

impl ByteStreamResponseType for AnnouncementsResponse {}

impl HTTPResponseType for AnnouncementsResponse {
    type ParsedResponseType =
        std::pin::Pin<Box<dyn futures_core::Stream<Item = reqwest::Result<prost::bytes::Bytes>> + Send>>;

    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        Ok(Box::pin(resp.bytes_stream()))
    }
}
