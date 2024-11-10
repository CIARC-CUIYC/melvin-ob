use std::error::Error as StdError;
use super::http_request::request_common::{HTTPRequestType, HTTPRequest};

#[derive(Debug)]
pub struct HTTPClient {
    client: reqwest::Client,
    base_url: String,
}

impl HTTPClient {
    pub fn new(base_url: &str) -> HTTPClient {
        HTTPClient {
            client: reqwest::Client::builder().danger_accept_invalid_certs(true).build().unwrap(),
            base_url: String::from(base_url),
        }
    }

    pub async fn execute_request<T>(&self, request: HTTPRequest<T>) -> Result<T::Response, Box<dyn StdError>>
    where
        T: HTTPRequestType,
    {
        match request {
            HTTPRequest::Get(get_request) =>
                self.get::<T::Response>(get_request.endpoint(), get_request.header_params()).await,
            HTTPRequest::Post(post_request) =>
                self.post::<T::Response, T::Body>(post_request.endpoint(), post_request.body(), post_request.header_params()).await,
            HTTPRequest::Put(put_request) =>
                self.put::<T::Response, T::Body>(put_request.endpoint(), put_request.body(), put_request.header_params()).await,
            HTTPRequest::Delete(delete_request) =>
                self.delete::<T::Response>(delete_request.endpoint(), delete_request.header_params()).await,
        }
    }

    async fn get<T>(&self, endpoint: &str, header: reqwest::header::HeaderMap) -> Result<T, Box<dyn StdError>>
    where
        T: for<'de> serde::Deserialize<'de>,
    {
        let url = format!("{}{}", self.base_url, endpoint);
        let response = self.client.get(&url).headers(header).send().await?;

        if response.status().is_success() {
            let data = response.json::<T>().await?;
            Ok(data)
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("GET ({}) failed: {}", endpoint, response.status()),
            )))
        }
    }

    async fn post<T, B>(&self, endpoint: &str, body: &B, header: reqwest::header::HeaderMap) -> Result<T, Box<dyn StdError>>
    where
        T: for<'de> serde::Deserialize<'de>,
        B: serde::Serialize,
    {
        let url = format!("{}{}", self.base_url, endpoint);
        let response = self.client.post(&url).json(body).headers(header).send().await?;

        if response.status().is_success() {
            let data = response.json::<T>().await?;
            Ok(data)
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("POST ({}) failed: {}", endpoint, response.status()),
            )))
        }
    }

    async fn put<T, B>(&self, endpoint: &str, body: &B, header: reqwest::header::HeaderMap) -> Result<T, Box<dyn StdError>>
    where
        T: for<'de> serde::Deserialize<'de>,
        B: serde::Serialize,
    {
        let url = format!("{}{}", self.base_url, endpoint);
        let response = self.client.put(&url).json(body).headers(header).send().await?;

        if response.status().is_success() {
            let data = response.json::<T>().await?;
            Ok(data)
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("PUT ({}) failed: {}", endpoint, response.status()),
            )))
        }
    }

    async fn delete<T>(&self, endpoint: &str, header: reqwest::header::HeaderMap) -> Result<T, Box<dyn StdError>>
    where
        T: for<'de> serde::Deserialize<'de>,
    {
        let url = format!("{}{}", self.base_url, endpoint);
        let response = self.client.delete(&url).headers(header).send().await?;

        if response.status().is_success() {
            let data = response.json::<T>().await?;
            Ok(data)
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("DELETE ({}) failed: {}", endpoint, response.status()),
            )))
        }
    }
}