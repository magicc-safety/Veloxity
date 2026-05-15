#![no_std]
pub mod board;
pub mod bodytype;
pub mod comm_manager;
pub mod comm_messages;
pub mod command_manager;
pub mod command_system;
pub mod companion_system;
pub mod control_system;
pub mod controller;
pub mod errors;
pub mod estimator;
pub mod events;
pub mod log_system;
pub mod mixer;
pub mod packets;
pub mod param_reactions;
pub mod param_system;
pub mod params;
pub mod ports;
pub mod pwm;
pub mod pwm_system;
pub mod rc;
pub mod rc_system;
pub mod sensor_health_system;
pub mod sensor_systems;
pub mod sensorprocessors;
pub mod sensors;
pub mod state_machine;
pub mod world;

pub use micro_algebra;

#[cfg(test)]
pub(crate) mod test_support;

// MAVLINK Specific
pub mod mavlink {
    include!(concat!(env!("OUT_DIR"), "/mavlink_generated/mod.rs"));
}

pub mod logger; // make the logging macros public
