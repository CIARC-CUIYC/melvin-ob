use crate::http_handler::http_response::response_common::{
    ByteStreamResponseType, HTTPResponseType, ResponseError,
};
use bytes::Bytes;
use futures::StreamExt;

pub struct ShootImageResponse {}

impl ByteStreamResponseType for ShootImageResponse {}

impl HTTPResponseType for ShootImageResponse {
    type ParsedResponseType = futures_core::stream::BoxStream<'static, reqwest::Result<Bytes>>;

    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        let stream = response.bytes_stream();
        Ok(stream.boxed())
    }
}
