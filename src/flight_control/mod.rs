//! This module provides core components and functionality for the flight system,
//! including the flight computer, flight state management, orbit calculations, 
//! and supervision logic.

mod flight_computer;
mod flight_state;
pub(crate) mod orbit;
mod supervisor;

pub use flight_computer::FlightComputer;
pub use flight_state::FlightState;
pub use supervisor::Supervisor;