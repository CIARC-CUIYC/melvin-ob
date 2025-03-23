use crate::console_communication::ConsoleMessenger;
use crate::flight_control::{
    camera_controller::CameraController, flight_computer::FlightComputer, orbit::ClosedOrbit,
    task::TaskController,
};
use crate::http_handler::http_client::HTTPClient;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Struct representing the key components of the application, providing access
/// to various subsystems such as the HTTP client, camera controller, flight computer,
/// task controller, and console messenger.
#[derive(Clone)]
pub struct Keychain {
    /// The HTTP client for performing network requests.
    client: Arc<HTTPClient>,
    /// The console messenger for handling console-related operations.
    con: Arc<ConsoleMessenger>,
    /// The flight computer responsible for managing satellite operations.
    f_cont: Arc<RwLock<FlightComputer>>,
    /// The task controller for scheduling and managing tasks.
    t_cont: Arc<TaskController>,
    /// The camera controller for handling camera-related operations.
    c_cont: Arc<CameraController>,
}

impl Keychain {
    /// Creates a new instance of `Keychain` asynchronously.
    ///
    /// # Arguments
    /// - `url`: The base URL to initialize the HTTP client.
    ///
    /// # Returns
    /// A new instance of `Keychain` containing initialized subsystems.
    pub async fn new(url: &str) -> Self {
        let client = Arc::new(HTTPClient::new(url));
        let c_cont = Arc::new(CameraController::start(
            "./".to_string(),
            Arc::clone(&client),
        ));
        let t_cont = Arc::new(TaskController::new());
        let con = Arc::new(ConsoleMessenger::start(
            Arc::clone(&c_cont),
            Arc::clone(&t_cont),
        ));
        let f_cont = Arc::new(RwLock::new(FlightComputer::new(Arc::clone(&client)).await));
        Self {
            client,
            con,
            f_cont,
            t_cont,
            c_cont,
        }
    }

    /// Provides a cloned reference to the HTTP client.
    pub fn client(&self) -> Arc<HTTPClient> { Arc::clone(&self.client) }

    /// Provides a cloned reference to the flight computer.
    pub fn f_cont(&self) -> Arc<RwLock<FlightComputer>> { Arc::clone(&self.f_cont) }

    /// Provides a cloned reference to the task controller.
    pub fn t_cont(&self) -> Arc<TaskController> { Arc::clone(&self.t_cont) }

    /// Provides a cloned reference to the console messenger.
    pub fn con(&self) -> Arc<ConsoleMessenger> { Arc::clone(&self.con) }

    /// Provides a cloned reference to the camera controller.
    pub fn c_cont(&self) -> Arc<CameraController> { Arc::clone(&self.c_cont) }
}

/// Struct representing an enhanced `Keychain` that includes a `ClosedOrbit`.
/// This struct offers access to various subsystems in addition to holding the orbit
/// and its related operations.
#[derive(Clone)]
pub struct KeychainWithOrbit {
    /// The HTTP client for performing network requests.
    client: Arc<HTTPClient>,
    /// The console messenger for handling console-related operations.
    con: Arc<ConsoleMessenger>,
    /// The flight computer responsible for managing satellite operations.
    f_cont: Arc<RwLock<FlightComputer>>,
    /// The task controller for scheduling and managing tasks.
    t_cont: Arc<TaskController>,
    /// The camera controller for handling camera-related operations.
    c_cont: Arc<CameraController>,
    /// The closed orbit object, protected by a read-write lock for thread-safe access.
    c_orbit: Arc<RwLock<ClosedOrbit>>,
}

impl KeychainWithOrbit {
    /// Creates a new instance of `KeychainWithOrbit` by combining an existing
    /// keychain and a closed orbit.
    ///
    /// # Arguments
    /// - `keychain`: The existing `Keychain` instance to extend.
    /// - `orbit`: The `ClosedOrbit` object to manage in the new instance.
    ///
    /// # Returns
    /// A new instance of `KeychainWithOrbit` containing the provided keychain subsystems
    /// and the closed orbit.
    pub fn new(keychain: Keychain, orbit: ClosedOrbit) -> Self {
        Self {
            client: keychain.client,
            con: keychain.con,
            f_cont: keychain.f_cont,
            t_cont: keychain.t_cont,
            c_cont: keychain.c_cont,
            c_orbit: Arc::new(RwLock::new(orbit)),
        }
    }

    /// Provides a cloned reference to the HTTP client.
    ///
    /// # Returns
    /// A thread-safe reference to the `HTTPClient`.
    pub fn client(&self) -> Arc<HTTPClient> { Arc::clone(&self.client) }

    /// Provides a cloned reference to the flight computer.
    ///
    /// # Returns
    /// A thread-safe reference to the `FlightComputer`.
    pub fn f_cont(&self) -> Arc<RwLock<FlightComputer>> { Arc::clone(&self.f_cont) }

    /// Provides a cloned reference to the task controller.
    ///
    /// # Returns
    /// A thread-safe reference to the `TaskController`.
    pub fn t_cont(&self) -> Arc<TaskController> { Arc::clone(&self.t_cont) }

    /// Provides a cloned reference to the camera controller.
    ///
    /// # Returns
    /// A thread-safe reference to the `CameraController`.
    pub fn c_cont(&self) -> Arc<CameraController> { Arc::clone(&self.c_cont) }

    /// Provides a cloned reference to the closed orbit.
    ///
    /// # Returns
    /// A thread-safe reference to the `ClosedOrbit`.
    pub fn c_orbit(&self) -> Arc<RwLock<ClosedOrbit>> { Arc::clone(&self.c_orbit) }

    /// Provides a cloned reference to the console messenger.
    ///
    /// # Returns
    /// A thread-safe reference to the `ConsoleMessenger`.
    pub fn con(&self) -> Arc<ConsoleMessenger> { Arc::clone(&self.con) }
}
