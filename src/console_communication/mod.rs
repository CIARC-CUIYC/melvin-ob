//! This module provides the main components for handling communication with the console.
//! It includes the `console_endpoint` module for managing console endpoints,
//! the `console_messenger` module for messaging functionality,
//! and the `melvin_messages` module for defining message structures and protocols.

mod console_endpoint;
mod console_messenger;
mod melvin_messages;

pub use console_messenger::ConsoleMessenger;
