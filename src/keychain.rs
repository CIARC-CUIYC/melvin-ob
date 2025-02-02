use crate::console_communication::console_messenger::ConsoleMessenger;
use crate::flight_control::{
    camera_controller::CameraController, flight_computer::FlightComputer,
    orbit::closed_orbit::ClosedOrbit, task::task_controller::TaskController,
};
use crate::http_handler::http_client::HTTPClient;
use std::sync::Arc;
use tokio::sync::RwLock;

#[derive(Clone)]
pub struct Keychain {
    client: Arc<HTTPClient>,
    con: Arc<ConsoleMessenger>,
    f_cont: Arc<RwLock<FlightComputer>>,
    t_cont: Arc<TaskController>,
    c_cont: Arc<CameraController>,
}

impl Keychain {
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

    pub fn client(&self) -> Arc<HTTPClient> { Arc::clone(&self.client) }
    pub fn f_cont(&self) -> Arc<RwLock<FlightComputer>> { Arc::clone(&self.f_cont) }
    pub fn t_cont(&self) -> Arc<TaskController> { Arc::clone(&self.t_cont) }
    pub fn con(&self) -> Arc<ConsoleMessenger> { Arc::clone(&self.con) }
    pub fn c_cont(&self) -> Arc<CameraController> { Arc::clone(&self.c_cont) }
}

#[derive(Clone)]
pub struct KeychainWithOrbit {
    client: Arc<HTTPClient>,
    con: Arc<ConsoleMessenger>,
    f_cont: Arc<RwLock<FlightComputer>>,
    t_cont: Arc<TaskController>,
    c_cont: Arc<CameraController>,
    c_orbit: Arc<RwLock<ClosedOrbit>>,
}

impl KeychainWithOrbit {
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

    pub fn client(&self) -> Arc<HTTPClient> { Arc::clone(&self.client) }
    pub fn f_cont(&self) -> Arc<RwLock<FlightComputer>> { Arc::clone(&self.f_cont) }
    pub fn t_cont(&self) -> Arc<TaskController> { Arc::clone(&self.t_cont) }
    pub fn c_cont(&self) -> Arc<CameraController> { Arc::clone(&self.c_cont) }
    pub fn c_orbit(&self) -> Arc<RwLock<ClosedOrbit>> { Arc::clone(&self.c_orbit) }
    pub fn con(&self) -> Arc<ConsoleMessenger> { Arc::clone(&self.con) }
}
