use super::{comm_link_trait::*, CommMessage, super::board::Board};
use super::*;
use super::comm_message_defs::*;
use micro_algebra::stack::{quaternion::Quaternion, vector::Vector};

pub struct Temp_Mavlink_Message_Type;
pub struct Temp_Mavlink_Status_Type;

pub struct Mavlink {
    compid: u32,
    in_buf: Temp_Mavlink_Message_Type,
    status: Temp_Mavlink_Status_Type,
    initialized: bool,
}

impl Mavlink {
    pub fn new() -> Self {
        Self {
            compid: 250,
            in_buf: Temp_Mavlink_Message_Type,
            status: Temp_Mavlink_Status_Type,
            initialized: false,
        }
    }

    pub fn handle_msg_param_request_list(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_msg_param_request_read(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_msg_param_set(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_msg_offboard_control(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_msg_external_attitude(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_msg_rosflight_cmd(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_msg_rosflight_aux_cmd(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_msg_timesync(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_msg_heartbeat(msg: Temp_Mavlink_Message_Type, message: CommMessage) {}
    pub fn handle_mavlink_message(msg: Temp_Mavlink_Message_Type, message: CommMessage) -> bool {true}
}

impl ListenerInterface for Mavlink {
    fn init(&mut self, baud_rate: ParamValue, dev: ParamValue, board: &mut dyn Board) {
        board.serial_init(u32::try_from(baud_rate).unwrap(), u32::try_from(dev).unwrap());
        self.initialized = true;
    }

    fn parse_char(&mut self, ch: u8, message: &CommMessage) {}
    fn send_attitude_quaternion(&mut self, system_id: u8, timestamp_us: u64, attitude: &Quaternion<f64>, angular_velocity: &Vector<f64, 3>) {}
    fn send_baro(&mut self, sysem_id: u8, altitude: f32, pressure: f32, temperature: f32) {}
    fn send_command_ack(&mut self, system_id: u8, command: CommMessageCommand, success: RosflightCmdResponse) {}
    fn send_diff_pressure(&mut self, system_id: u8, velocity: f32, pressure: f32, temperature: f32) {}
    fn send_heartbeat(&mut self, system_id: u8, fixed_wing: bool) {}
    fn send_imu(&mut self, system_id: u8, timestamp_us: u64, accel: &Vector<f64, 3>, gyro: &Vector<f64, 3>, temperature: f32) {}
    fn send_log_message(&mut self, system_id: u8, severity: LogSeverity, text: [u8; LOG_MESSAGE_SIZE]) {}
    fn send_mag(&mut self, system_id: u8, mag: &Vector<f64, 3>) {}
    fn send_named_value(&mut self, system_id: u8, timestamp_ms: u32, name: [u8; PARAM_NAME_LENGTH], name_length: usize, value: ParamValue) {}
    fn send_output_raw(&mut self, system_id: u8, timestamp_ms: u32, raw_outputs: [f32; 14]) {}
    fn send_param_value(&mut self, system_id: u8, name: [u8; PARAM_NAME_LENGTH], value: ParamValue) {} // this will need to be changed as we change the XML representation to fit with our params representation and how it differs from C++
    fn send_rc_raw(&mut self, system_id: u8, timestamp_ms: u32, channels: [u16; 8]) {}
    fn send_sonar(&mut self, system_id: u8, type_: u8, range: f32, max_range: f32, min_range: f32) {}
    fn send_status(&mut self, system_id: u8, armed: bool, failsafe: bool, rc_override: bool, offboard: bool, error_code: u8, control_mode: u8, num_errors: i16, loop_time_us: i16) {}
    fn send_timesync(&mut self, system_id: u8, tc1: i64, ts1: i64) {}
    fn send_version(&mut self, system_id: u8, version: [u8; PARAM_NAME_LENGTH]) {}
    fn send_gnss(&mut self, system_id: u8, data: &Temp_GNSSData) {}
    fn send_gnss_full(&mut self, system_id: u8, data: &Temp_GNSSFull) {}
    fn send_error_data(&mut self, system_id: u8, error_data: &Temp_BackupData) {}
    fn send_battery_status(&mut self, system_id: u8, voltage: f32, current: f32) {}
}