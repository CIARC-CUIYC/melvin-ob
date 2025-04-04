//! This module organizes and exposes various operational modes for the system, 
//! including the abstract global mode trait, in orbit mode, and zoned objective preparation/
//! retrieval modes. Each mode is implemented in its respective submodule.

mod global_mode;
mod in_orbit_mode;
mod orbit_return_mode;
mod zo_prep_mode;
mod zo_retrieval_mode;

pub(crate) use orbit_return_mode::OrbitReturnMode;
pub(crate) use global_mode::GlobalMode;