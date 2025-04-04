//! This module provides core structs, enums, and utilities for interacting with the DRS backend system.
//! It includes functionalities such as retrieving the objective list or the most recent observation.

mod common;
pub mod http_client;
pub mod http_request;
pub mod http_response;

pub use common::BeaconObjective;
pub use common::HTTPError;
pub(crate) use common::ImageObjective;
pub(crate) use common::ZoneType;