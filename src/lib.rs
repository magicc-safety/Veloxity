#![cfg_attr(not(feature = "use_std"), no_std)] // if the feature "use_std" is not enabled, definitely turn off the entire std environment using the compiler directive "no_std"

pub(crate) mod units;
pub mod board;
pub mod params;
pub mod comm_manager;
pub mod rustflight;
pub mod sensors;
mod state_machine;