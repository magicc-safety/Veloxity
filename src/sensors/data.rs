use crate::units::{
    MM, CM, UnsignedMM, UnsignedCM, CMPerSec, UnsignedCMPerSec, DegENeg7,
    UnixTimeSeconds, FracTime, MMPerSec, ROSFlightTimestamp, TimeOfWeek,
    Year, Month, Day, Hour, Minute, Second, Nanosecond, Longitude,
    Latitude, Meter, MeterPerSec, Radians,
};

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


/*
TODO: Fix type values taken from MAVLink; change if necessary
https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
 */
#[derive(Default, Debug)]
pub enum FixType {
    #[default] // Sets default field to NoGPS
    NoGPS,
    NoFix,
    Fix2D,
    Fix3D,
    DGPS,
    RTKFloat,
    RTKFixed,
    Static,
    PPP,
}

/*
Many of these fields seem to have been partially taken or inspired by
related MAVLink fields.

TODO: Figure out units for these fields.
 */
#[derive(Default)]
pub struct GNSSFull {
    time_of_week: TimeOfWeek,
    year: Year,
    month: Month,
    day: Day,
    hour: Hour,
    min: Minute,
    sec: Second,
    valid: u8, // Units?
    t_acc: i32, // Units?
    nano: Nanosecond,
    fix_type: FixType,
    num_sat: u8,
    lon: Longitude,
    lat: Latitude,
    height: Meter, // Units? MAVLink specifies meters
    height_msl: Meter, // Units?
    h_acc: Meter, // Units?
    v_acc: MeterPerSec, // Units?
    vel_n: MeterPerSec,
    vel_e: MeterPerSec,
    vel_d: MeterPerSec,
    g_speed: MeterPerSec, // Units? This seems to be ground speed
    head_mot: i32, // Units? Not sure what this field represents
    s_acc: Second, // Units? Seems to be seconds accuracy...?
    head_acc: Radians, // Units?
    p_dop: u16, // Units? Not sure what this represents
    rosflight_timestamp: ROSFlightTimestamp, // microseconds, time stamp of last byte in the message
}



