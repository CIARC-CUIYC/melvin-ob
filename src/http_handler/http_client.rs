/// A simple wrapper around `reqwest::Client` used to manage HTTP requests
/// with a preconfigured base URL and default settings.
///
/// This client is used for making REST API calls to the DRS backend.
/// It sets a fixed timeout and allows easy reuse of the HTTP client infrastructure.
#[derive(Debug)]
pub(crate) struct HTTPClient {
    /// The underlying `reqwest::Client` used to perform HTTP requests.
    client: reqwest::Client,
    /// Base URL for the API, prepended to all endpoint paths. 
    base_url: String,
}

impl HTTPClient {
    /// Constructs a new `HTTPClient` with the given base URL.
    ///
    /// This client has a default request timeout of 5 seconds.
    ///
    /// # Arguments
    /// * `base_url` – The root URL for all HTTP requests (e.g., `"http://localhost:8000/api"`).
    ///
    /// # Returns
    /// A configured `HTTPClient` instance.
    pub(crate) fn new(base_url: &str) -> HTTPClient {
        HTTPClient {
            client: reqwest::Client::builder()
                //.danger_accept_invalid_certs(true)
                .timeout(std::time::Duration::from_secs(5))
                .build()
                .unwrap(),
            base_url: String::from(base_url),
        }
    }

    /// Returns a reference to the internal `reqwest::Client`.
    pub(super) fn client(&self) -> &reqwest::Client { &self.client }
    /// Returns the base URL that the client was initialized with.
    pub(crate) fn url(&self) -> &str { self.base_url.as_str() }
}
