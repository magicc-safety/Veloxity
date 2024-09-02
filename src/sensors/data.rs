use crate::units::{CM, UnsignedCM, CMPerSec, UnsignedCMPerSec, DegENeg7};

// Newtype pattern for units to prevent conversion errors.

#[derive(Default)]
struct ECEF {
    x: CM,
    y: CM,
    z: CM,
    p_accel: UnsignedCM,
    vx: CMPerSec,
    vy: CMPerSec,
    vz: CMPerSec,
    s_accel: UnsignedCMPerSec,
}

#[derive(Default)]
struct GNSSData {
    // fix_type: GNSSFixType, // TODO: Create this
    time_of_week: u32,
    time: u64, // TODO: This is Unix time in seconds; create a type for this

}

