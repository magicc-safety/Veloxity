use super::*;
use super::super::board::Board;
use micro_algebra::stack::{quaternion::Quaternion, vector::Vector};

pub struct Temp_ECEF {
    x: i32,
    y: i32,
    z: i32,
    p_acc: u32,
    vx: i32,
    vy: i32,
    vz: i32,
    s_acc: u32,
}

pub enum Temp_GNSSFixType {
    GNSSFixTypeNoFix,
    GNSSFixTypeDeadReckoningOnly,
    GNSSFixType2DFix,
    GNSSFixType3DFix,
    GnssFixTypeGNSSPlusDeadReckoning,
    GnssFixTypeTimeFixOnly,
}

pub struct Temp_GNSSData {
    fix_type: Temp_GNSSFixType,
    ecef: Temp_ECEF,
    time_of_week: u32,
    time: u64,
    nanos: u64,
    lat: i32,
    lon: i32,
    height: i32,
    vel_n: i32,
    vel_e: i32,
    vel_d: i32,
    h_acc: u32,
    v_acc: u32,
    rosflight_timestamp: u32,
}

pub struct Temp_GNSSFull {
    time_of_week: u64,
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    min: u8,
    sec: u8,
    valid: u8,
    t_acc: u32,
    nano: i32,
    fix_type: u8,
    lon: i32,
    lat: i32,
    height: i32,
    height_msl: i32,
    h_acc: u32,
    v_acc: u32,
    vel_n: i32,
    vel_e: i32,
    vel_d: i32,
    g_speed: i32,
    head_mot: i32,
    s_acc: u32,
    head_acc: u32,
    p_dop: u16,
    rosflight_timestamp: u64, // microseconds, time stamp of last byte in the message
}

pub struct Temp_DebugInfo {
    r0: u32,
    r1: u32,
    r2: u32,
    r3: u32,
    r12: u32,
    lr: u32,
    pc: u32,
    psr: u32,
}

impl Temp_DebugInfo {
    pub fn new() -> Self {
        Temp_DebugInfo {
            r0: 0,
            r1: 0,
            r2: 0,
            r3: 0,
            r12: 0,
            lr: 0,
            pc: 0,
            psr: 0,
        }
    }
}

pub struct Temp_BackupData {
    ARM_MAGIC: u32,
    reset_count: u16,
    error_code: u16,
    arm_flag: u32,
    debug: Temp_DebugInfo,
    checksum: u32,
}

impl Temp_BackupData {
    pub fn new() -> Self {
        Temp_BackupData {
            ARM_MAGIC: 0,
            reset_count: 0,
            error_code: 0,
            arm_flag: 0,
            debug: Temp_DebugInfo::new(),
            checksum: 0,
        }
    }
}

pub trait ListenerInterface {
    fn init(&mut self, baud_rate: ParamValue, dev: ParamValue, board: &mut dyn Board);
    fn parse_char(&mut self, ch: u8, message: &CommMessage);
    fn send_attitude_quaternion(&mut self, system_id: u8, timestamp_us: u64, attitude: &Quaternion<f64>, angular_velocity: &Vector<f64, 3>);
    fn send_baro(&mut self, sysem_id: u8, altitude: f32, pressure: f32, temperature: f32);
    fn send_command_ack(&mut self, system_id: u8, command: CommMessageCommand, success: RosflightCmdResponse);
    fn send_diff_pressure(&mut self, system_id: u8, velocity: f32, pressure: f32, temperature: f32);
    fn send_heartbeat(&mut self, system_id: u8, fixed_wing: bool);
    fn send_imu(&mut self, system_id: u8, timestamp_us: u64, accel: &Vector<f64, 3>, gyro: &Vector<f64, 3>, temperature: f32);
    fn send_log_message(&mut self, system_id: u8, severity: LogSeverity, text: [u8; LOG_MESSAGE_SIZE]);
    fn send_mag(&mut self, system_id: u8, mag: &Vector<f64, 3>);
    fn send_named_value(&mut self, system_id: u8, timestamp_ms: u32, name: [u8; PARAM_NAME_LENGTH], name_length: usize, value: ParamValue);
    fn send_output_raw(&mut self, system_id: u8, timestamp_ms: u32, raw_outputs: [f32; 14]);
    fn send_param_value(&mut self, system_id: u8, name: [u8; PARAM_NAME_LENGTH], value: ParamValue); // this will need to be changed as we change the XML representation to fit with our params representation and how it differs from C++
    fn send_rc_raw(&mut self, system_id: u8, timestamp_ms: u32, channels: [u16; 8]);
    fn send_sonar(&mut self, system_id: u8, type_: u8, range: f32, max_range: f32, min_range: f32);
    fn send_status(&mut self, system_id: u8, armed: bool, failsafe: bool, rc_override: bool, offboard: bool, error_code: u8, control_mode: u8, num_errors: i16, loop_time_us: i16);
    fn send_timesync(&mut self, system_id: u8, tc1: i64, ts1: i64);
    fn send_version(&mut self, system_id: u8, version: [u8; PARAM_NAME_LENGTH]);
    fn send_gnss(&mut self, system_id: u8, data: &Temp_GNSSData);
    fn send_gnss_full(&mut self, system_id: u8, data: &Temp_GNSSFull);
    fn send_error_data(&mut self, system_id: u8, error_data: &Temp_BackupData);
    fn send_battery_status(&mut self, system_id: u8, voltage: f32, current: f32);
}