use crate::units::{MM, CM, UnsignedMM, UnsignedCM, CMPerSec, UnsignedCMPerSec, DegENeg7, UnixTimeSeconds, FracTime, MMPerSec,ROSFlightTimestamp};

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
    time_of_week: u32, // TODO: What unit should this be?
    time: UnixTimeSeconds,
    nanos: FracTime,
    lat: DegENeg7,
    lon: DegENeg7,
    height: MM,
    vel_n: MMPerSec,
    vel_e: MMPerSec,
    vel_d: MMPerSec,
    h_acc: MM,
    v_acc: MM, // TODO: Is this correct units?
    ECEF: ECEF,
    rosflight_timestamp: ROSFlightTimestamp,
}


#[derive(Default)]
pub struct GNSSFull {
    uint64_t time_of_week;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t t_acc;
    int32_t nano;
    uint8_t fix_type;
    uint8_t num_sat;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t height_msl;
    uint32_t h_acc;
    uint32_t v_acc;
    int32_t vel_n;
    int32_t vel_e;
    int32_t vel_d;
    int32_t g_speed;
    int32_t head_mot;
    uint32_t s_acc;
    uint32_t head_acc;
    uint16_t p_dop;
    uint64_t rosflight_timestamp; // microseconds, time stamp of last byte in the message

}



