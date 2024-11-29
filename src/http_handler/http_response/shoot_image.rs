use crate::http_handler::http_response::response_common::{
    ByteStreamResponseType, HTTPResponseType, ResponseError,
};

pub struct ShootImageResponse {}

impl ByteStreamResponseType for ShootImageResponse {}

impl HTTPResponseType for ShootImageResponse {
    type ParsedResponseType =
        std::pin::Pin<Box<dyn futures_core::Stream<Item = reqwest::Result<bytes::Bytes>> + Send>>;

    async fn read_response(
        response: reqwest::Response,
    ) -> Result<Self::ParsedResponseType, ResponseError> {
        let response = Self::unwrap_return_code(response).await?;
        Ok(Box::pin(response.bytes_stream()))
    }
}
