use serde::{Deserialize, Serialize};

// ============ Standard ROS Types ============

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

// ============ ROSflight Messages ============

// Airspeed.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Airspeed {
    pub header: Header,
    pub differential_pressure: f64,
    pub temperature: f64,
}

// Attitude.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Attitude {
    pub header: Header,
    pub quaternion: Quaternion,
    pub angular_velocity: Vector3,
}

// AuxCommand.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct AuxCommand {
    pub header: Header,
    pub type_array: Vec<u8>,
    pub values: Vec<f32>,
}

impl AuxCommand {
    pub const AUX_COMMAND_DISABLED: u8 = 0;
    pub const AUX_COMMAND_SERVO: u8 = 1;
    pub const AUX_COMMAND_MOTOR: u8 = 2;
    pub const NUM_AUX_CHANNELS: u8 = 14;
}

// Barometer.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Barometer {
    pub header: Header,
    pub pressure: f64,
    pub temperature: f64,
}

// BatteryStatus.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct BatteryStatus {
    pub header: Header,
    pub voltage: f32,
    pub current: f32,
}

// Command.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Command {
    pub header: Header,
    pub mode: u8,
    pub ignore: u8,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub f: f32,
}

impl Command {
    pub const MODE_PASS_THROUGH: u8 = 0;
    pub const MODE_ROLL_PITCH_YAWRATE_THROTTLE: u8 = 1;
    pub const MODE_ROLL_PITCH_YAWRATE_ALTITUDE: u8 = 2;

    pub const IGNORE_NONE: u8 = 0;
    pub const IGNORE_X: u8 = 1;
    pub const IGNORE_Y: u8 = 2;
    pub const IGNORE_Z: u8 = 4;
    pub const IGNORE_F: u8 = 8;
}

// Error.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Error {
    pub header: Header,
    pub error_code: u8,
    pub error_message: String,
    pub on_error: bool,
    pub rearm: bool,
}

// GNSS.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct GNSS {
    pub header: Header,
    pub fix_type: u8,
    pub num_sat: u8,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub height_msl: i32,
    pub h_acc: u32,
    pub v_acc: u32,
    pub vel_n: i32,
    pub vel_e: i32,
    pub vel_d: i32,
    pub speed_accuracy: u32,
}

impl GNSS {
    pub const GNSS_FIX_TYPE_NO_FIX: u8 = 0;
    pub const GNSS_FIX_TYPE_DEAD_RECKONING: u8 = 1;
    pub const GNSS_FIX_TYPE_2D_FIX: u8 = 2;
    pub const GNSS_FIX_TYPE_3D_FIX: u8 = 3;
    pub const GNSS_FIX_TYPE_GPS_DEAD_RECKONING: u8 = 4;
    pub const GNSS_FIX_TYPE_TIME_ONLY: u8 = 5;
}

// OutputRaw.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct OutputRaw {
    pub header: Header,
    pub values: Vec<f32>,
}

// RCRaw.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct RCRaw {
    pub header: Header,
    pub values: Vec<u16>,
}

// Status.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Status {
    pub header: Header,
    pub armed: bool,
    pub failsafe: bool,
    pub rc_override: bool,
    pub offboard: bool,
    pub control_mode: u8,
    pub num_errors: u8,
    pub loop_time_us: u16,
}

impl Status {
    pub const CONTROL_MODE_ANGLE: u8 = 0;
    pub const CONTROL_MODE_RATE: u8 = 1;
    pub const CONTROL_MODE_PASS_THROUGH: u8 = 2;
}

// ============ ROSflight Services ============

// ParamFile.srv
pub mod param_file {
    use super::*;

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Request {
        pub filename: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Response {
        pub success: bool,
    }
}

// ParamGet.srv
pub mod param_get {
    use super::*;

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Request {
        pub name: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Response {
        pub exists: bool,
        pub value: f64,
    }
}

// ParamSet.srv
pub mod param_set {
    use super::*;

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Request {
        pub name: String,
        pub value: f64,
    }

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Response {
        pub exists: bool,
    }
}
