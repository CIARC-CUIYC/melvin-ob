use crate::http_handler::http_response::response_common::{
    ByteStreamResponseType, HTTPResponseType, ResponseError,
};
use futures::StreamExt;
use prost::bytes::Bytes;

/// Response type for the /image endpoint -> GET
pub struct ShootImageResponse {}

impl ByteStreamResponseType for ShootImageResponse {}

impl HTTPResponseType for ShootImageResponse {
    /// Parsed type of the response
    type ParsedResponseType = futures_core::stream::BoxStream<'static, reqwest::Result<Bytes>>;

    /// Return the deserialized response as a boxed byte stream 
    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let resp = Self::unwrap_return_code(response).await?;
        let stream = resp.bytes_stream();
        Ok(stream.boxed())
    }
}
