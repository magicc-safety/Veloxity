#![no_std]

mod conversions;
pub mod link;
pub mod parser;

pub use link::MavlinkInterface;

pub mod generated {
    include!(concat!(env!("OUT_DIR"), "/mavlink_generated/mod.rs"));
}
