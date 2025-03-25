mod message_logger;
mod comm_message_defs;
mod comm_link_trait;
pub mod mavlink;
use message_logger::*;
use comm_message_defs::*;
use comm_link_trait::*;
use crate::board::Board;

use super::params::*;

enum StreamId {
    Heartbeat,
    Status,
    Attitude,
    Imu,
    DiffPressure,
    Baro,
    Sonar,
    Mag,
    BatteryStatus,
    ServoOutputRaw,
    GNSS,
    GNSSFull,
    RcRaw,
    LowPriority,
    End,
}

pub struct CommManager<T: ListenerInterface> {
    sysid: u8,
    offboard_control_time: u64,
    initialized: bool,
    connected: bool,
    log_buffer: LogMessageBuffer,
    backup_data_buffer: Temp_BackupData,
    have_backup_data: bool,
    last_sent_gnss_tow: u32,
    last_sent_gnss_full_tow: u32,

    comm_link: T,
}

impl<T: ListenerInterface> CommManager<T> {
    pub fn new(comm_link: T) -> Self {
        CommManager {
            sysid: 0,
            offboard_control_time: 0,
            initialized: false,
            connected: false,
            log_buffer: LogMessageBuffer::new(),
            backup_data_buffer: Temp_BackupData::new(),
            have_backup_data: false,
            last_sent_gnss_full_tow: 0,
            last_sent_gnss_tow: 0,

            comm_link,
        }
    }
    
    fn update_system_id(&mut self, param_id: ParamValue) {
        if let ParamValue::Int(t) = param_id {
            self.sysid = t as u8;
        } 
    }

    fn send_heartbeat(&mut self) {
        self.comm_link.send_heartbeat(self.sysid, true);
    }

    fn send_status(&mut self) {
        // self.comm_link.send_status(self.sysid, armed, failsafe, rc_override, offboard, error_code, control_mode, num_errors, loop_time_us);
    }

    fn send_attitude(&mut self) {
        // self.comm_link.send_attitude_quaternion(self.sysid, timestamp_us, attitude, angular_velocity);
    }

    fn send_imu(&mut self) {
        // self.comm_link.send_imu(self.sysid, timestamp_us, accel, gyro, temperature);
    }

    fn send_output_raw(&mut self) {
        // self.comm_link.send_output_raw(self.sysid, timestamp_ms, raw_outputs);
    }

    fn send_rc_raw(&mut self) {
        // self.comm_link.send_rc_raw(self.sysid, timestamp_ms, channels);
    }

    fn send_diff_pressure(&mut self) {
        // self.comm_link.send_diff_pressure(self.sysid, velocity, pressure, temperature);
    }

    pub fn send_baro(&mut self) {
        // self.comm_link.send_baro(self.sysid, altitude, pressure, temperature);
    }

    fn send_sonar(&mut self) {
        // self.comm_link.send_sonar(self.sysid, type_, range, max_range, min_range);
    }

    pub fn send_mag(&mut self) {
        // self.comm_link.send_mag(self.sysid, mag);
    }

    fn send_battery_status(&mut self) {
        // self.comm_link.send_battery_status(self.sysid, voltage, current);
    }

    fn send_gnss(&mut self) {
        // self.comm_link.send_gnss(self.sysid, data);
    }

    fn send_gnss_full(&mut self) {
        // self.comm_link.send_gnss_full(self.sysid, data);
    }

    fn send_low_priority(&mut self) {
        // send buffered log messages 
        
        if self.connected && !self.log_buffer.empty() {
            let msg = self.log_buffer.oldest().expect("Something's wrong... I'm not supposed to panic");
            self.comm_link.send_log_message(self.sysid, msg.severity, msg.msg);
        }
    }

    fn send_rosflight_cmd_ack(&mut self) {
        // not defined in the c++ code either on the comms-refactor branch...
    }

    fn send_next_param() {} // probs going to avoid this one for now
    
    fn receive_msg_offboard_control(msg: &CommMessage) {}
    fn receive_msg_param_request_list(msg: &CommMessage) {}
    fn receive_msg_param_request_read(msg: &CommMessage) {}
    fn receive_msg_param_set(msg: &CommMessage) {}
    fn receive_msg_rosflight_cmd(msg: &CommMessage) {}
    fn receive_msg_rosflight_aux_cmd(msg: &CommMessage) {}
    fn receive_msg_timesync(msg: &CommMessage) {}
    fn receive_msg_external_attitude(msg: &CommMessage) {}
    fn receive_msg_heartbeat(msg: &CommMessage) {}
    
    pub fn init(&mut self, params: &Params, board: &mut dyn Board) {
        self.comm_link.init(*params.get_baud_rate(), *params.get_serial_device(), board);
        self.offboard_control_time = 0;
        self.update_system_id(*params.get_system_id());
        self.initialized = true;
    }

    pub fn receive(&mut self) {

    }

    pub fn stream(&mut self) {}
    pub fn send_param_value(&mut self) {}

    pub fn update_status(&mut self) {
        self.send_status();
    }

    pub fn log(&mut self, severity: LogSeverity, fmt: [u8; 230]) {} // fix magic number...
    pub fn log_message(&mut self, severity: LogSeverity, text: [u8; LOG_MESSAGE_SIZE]) {}
    pub fn send_named_value(&mut self, name: [u8; PARAM_NAME_LENGTH], value: ParamValue) {} // in c++ code, private for int, public for float...
    pub fn send_backup_data(&mut self, backup_data: Temp_BackupData) {}

}