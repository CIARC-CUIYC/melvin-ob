use reqwest::{Client, Request};
use serde::{Deserialize, Serialize};
use std::error::Error as StdError;
use super::http_request::{HTTPRequest, HTTPRequestType};

#[derive(Debug)]
pub struct HTTPClient {
    client: Client,
    base_url: String,
}

impl HTTPClient {
    pub fn new(base_url: &str) -> HTTPClient {
        HTTPClient {
            client: Client::new(),
            base_url: String::from(base_url),
        }
    }

    pub async fn execute_request<T>(&self, request: HTTPRequest<T>) -> Result<T::Response, Box<dyn StdError>>
    where T: HTTPRequestType
    {
        match request {
            HTTPRequest::Get(get_request) => 
                self.get::<T::Response>(get_request.endpoint()).await,
            HTTPRequest::Post(post_request) => 
                self.post::<T::Response, T::Body>(post_request.endpoint(), post_request.body()).await,
            HTTPRequest::Put(put_request) => 
                self.put::<T::Response, T::Body>(put_request.endpoint(), put_request.body()).await,
            HTTPRequest::Delete(delete_request) => 
                self.delete::<T::Response>(delete_request.endpoint()).await,
        }
    }

    async fn get<T>(&self, endpoint: &str) -> Result<T, Box<dyn StdError>>
    where T: for<'de> Deserialize<'de>,
    {
        let url = format!("{}{}", self.base_url, endpoint);
        let response = self.client.get(&url).send().await?;

        if response.status().is_success() {
            let data = response.json::<T>().await?;
            Ok(data)
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("GET failed: {}", response.status()),
            )))
        }
    }

    async fn post<T, B>(&self, endpoint: &str, body: &B) -> Result<T, Box<dyn StdError>>
    where
        T: for<'de> Deserialize<'de>,
        B: Serialize,
    {
        let url = format!("{}{}", self.base_url, endpoint);
        let response = self.client.post(&url).json(body).send().await?;

        if response.status().is_success() {
            let data = response.json::<T>().await?;
            Ok(data)
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("POST failed: {}", response.status()),
            )))
        }
    }

    async fn put<T, B>(&self, endpoint: &str, body: &B) -> Result<T, Box<dyn StdError>>
    where
        T: for<'de> Deserialize<'de>,
        B: Serialize,
    {
        let url = format!("{}{}", self.base_url, endpoint);
        let response = self.client.put(&url).json(body).send().await?;

        if response.status().is_success() {
            let data = response.json::<T>().await?;
            Ok(data)
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Failed to put data: {}", response.status()),
            )))
        }
    }

    async fn delete<T>(&self, endpoint: &str) -> Result<T, Box<dyn StdError>> 
    where T: for<'de> Deserialize<'de>
    {
        let url = format!("{}{}", self.base_url, endpoint);
        let response = self.client.delete(&url).send().await?;

        if response.status().is_success() {
            let data = response.json::<T>().await?;
            Ok(data)
        } else {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("DELETE failed: {}", response.status()),
            )))
        }
    }
}