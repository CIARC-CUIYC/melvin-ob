#[derive(Debug)]
pub struct HTTPClient {
    client: reqwest::Client,
    base_url: String,
}

impl HTTPClient {
    pub fn new(base_url: &str) -> HTTPClient {
        HTTPClient {
            client: reqwest::Client::builder()
                .danger_accept_invalid_certs(true)
                .timeout(std::time::Duration::from_secs(5))
                .build()
                .unwrap(),
            base_url: String::from(base_url),
        }
    }
    pub fn client(&self) -> &reqwest::Client {
        &self.client
    }
    pub fn url(&self) -> &str {
        self.base_url.as_str()
    }
}
