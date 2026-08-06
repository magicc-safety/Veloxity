use crate::{
    board::{BackupData, BoardIo},
    comm::interface::CommInterface,
    comm::messages::{Messages, messages::*},
    errors,
};

#[derive(Default)]
pub struct TestBoard {
    pub current_time_us: u64,
    pub tx_write_count: usize,
    pub sensor_errors_count: u16,
    pub backup_data: Option<BackupData>,
    pub backup_clear_count: usize,
    pub led0_high: bool,
    pub led1_high: bool,
    pub battery_configure_count: usize,
    pub battery_multipliers: Option<(f32, f32)>,
}

impl BoardIo for TestBoard {
    fn configure_battery_monitor(&mut self, voltage_multiplier: f32, current_multiplier: f32) {
        self.battery_configure_count += 1;
        self.battery_multipliers = Some((voltage_multiplier, current_multiplier));
    }

    fn serial_rx_read(&mut self, _buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        None
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        self.tx_write_count += 1;
        Some(Ok(bytes.len()))
    }

    fn clock_millis(&self) -> u32 {
        (self.current_time_us / 1000) as u32
    }

    fn clock_micros(&self) -> u64 {
        self.current_time_us
    }

    fn sensors_errors_count(&self) -> u16 {
        self.sensor_errors_count
    }

    fn led0_on(&mut self) {
        self.led0_high = true;
    }

    fn led0_off(&mut self) {
        self.led0_high = false;
    }

    fn led1_on(&mut self) {
        self.led1_high = true;
    }

    fn led1_off(&mut self) {
        self.led1_high = false;
    }

    fn backup_memory_read(&mut self) -> Option<BackupData> {
        self.backup_data.take()
    }

    fn backup_memory_clear(&mut self) -> bool {
        self.backup_clear_count += 1;
        true
    }
}

pub struct RecordingCommLink {
    pub sent_param_values: [Option<ParamValueMsg>; 8],
    pub sent_param_value_count: usize,
    pub heartbeat_count: usize,
    pub status_count: usize,
    pub last_status: Option<RosflightStatusMsg>,
    pub imu_count: usize,
    pub attitude_count: usize,
    pub baro_count: usize,
    pub diff_pressure_count: usize,
    pub mag_count: usize,
    pub range_count: usize,
    pub battery_count: usize,
    pub gnss_count: usize,
    pub last_imu: Option<SmallImuMsg>,
    pub last_baro: Option<SmallBaroMsg>,
    pub last_diff_pressure: Option<DiffPressureMsg>,
    pub last_range: Option<SmallRangeMsg>,
    pub last_gnss: Option<RosflightGnssMsg>,
    pub rc_channels_count: usize,
    pub last_rc_channels: Option<RcChannelsMsg>,
    pub output_raw_count: usize,
    pub last_output_raw: Option<RosflightOutputRawMsg>,
    pub timesync_count: usize,
    pub last_timesync: Option<TimesyncMsg>,
    pub version_count: usize,
    pub last_version: Option<RosflightVersionMsg>,
    pub cmd_ack_count: usize,
    pub last_cmd_ack: Option<RosflightCmdAckMsg>,
    pub statustext_count: usize,
    pub last_statustext: Option<StatustextMsg>,
    pub hard_error_count: usize,
    pub last_hard_error: Option<RosflightHardErrorMsg>,
}

impl RecordingCommLink {
    pub fn new() -> Self {
        Self {
            sent_param_values: [None; 8],
            sent_param_value_count: 0,
            heartbeat_count: 0,
            status_count: 0,
            last_status: None,
            imu_count: 0,
            attitude_count: 0,
            baro_count: 0,
            diff_pressure_count: 0,
            mag_count: 0,
            range_count: 0,
            battery_count: 0,
            gnss_count: 0,
            last_imu: None,
            last_baro: None,
            last_diff_pressure: None,
            last_range: None,
            last_gnss: None,
            rc_channels_count: 0,
            last_rc_channels: None,
            output_raw_count: 0,
            last_output_raw: None,
            timesync_count: 0,
            last_timesync: None,
            version_count: 0,
            last_version: None,
            cmd_ack_count: 0,
            last_cmd_ack: None,
            statustext_count: 0,
            last_statustext: None,
            hard_error_count: 0,
            last_hard_error: None,
        }
    }

    fn record_param_value(&mut self, msg: ParamValueMsg) {
        if self.sent_param_value_count < self.sent_param_values.len() {
            self.sent_param_values[self.sent_param_value_count] = Some(msg);
        }
        self.sent_param_value_count += 1;
    }
}

impl Default for RecordingCommLink {
    fn default() -> Self {
        Self::new()
    }
}

impl CommInterface<TestBoard> for RecordingCommLink {
    fn send_heartbeat(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        _msg: HeartbeatMsg,
    ) -> bool {
        self.heartbeat_count += 1;
        true
    }

    fn send_named_value(&mut self, _board: &mut TestBoard, _system_id: u8, msg: ParamValueMsg) {
        self.record_param_value(msg);
    }

    fn send_status(&mut self, _board: &mut TestBoard, _system_id: u8, msg: RosflightStatusMsg) {
        self.status_count += 1;
        self.last_status = Some(msg);
    }

    fn send_timesync(&mut self, _board: &mut TestBoard, _system_id: u8, msg: TimesyncMsg) -> bool {
        self.timesync_count += 1;
        self.last_timesync = Some(msg);
        true
    }

    fn send_version(&mut self, _board: &mut TestBoard, _system_id: u8, msg: RosflightVersionMsg) {
        self.version_count += 1;
        self.last_version = Some(msg);
    }

    fn send_output_raw(
        &mut self,
        _baord: &mut TestBoard,
        _system_id: u8,
        msg: RosflightOutputRawMsg,
    ) {
        self.output_raw_count += 1;
        self.last_output_raw = Some(msg);
    }

    fn send_attitude(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        _msg: AttitudeQuaternionMsg,
    ) {
        self.attitude_count += 1;
    }

    fn send_baro(&mut self, _board: &mut TestBoard, _system_id: u8, msg: SmallBaroMsg) {
        self.baro_count += 1;
        self.last_baro = Some(msg);
    }

    fn send_diff_pressure(&mut self, _board: &mut TestBoard, _system_id: u8, msg: DiffPressureMsg) {
        self.diff_pressure_count += 1;
        self.last_diff_pressure = Some(msg);
    }

    fn send_imu(&mut self, _board: &mut TestBoard, _system_id: u8, msg: SmallImuMsg) {
        self.imu_count += 1;
        self.last_imu = Some(msg);
    }

    fn send_mag(&mut self, _board: &mut TestBoard, _system_id: u8, _msg: SmallMagMsg) {
        self.mag_count += 1;
    }

    fn send_rc_raw(&mut self, _board: &mut TestBoard, _system_id: u8, msg: RcChannelsMsg) {
        self.rc_channels_count += 1;
        self.last_rc_channels = Some(msg);
    }

    fn send_range(&mut self, _board: &mut TestBoard, _system_id: u8, msg: SmallRangeMsg) {
        self.range_count += 1;
        self.last_range = Some(msg);
    }

    fn send_gnss(&mut self, _board: &mut TestBoard, _system_id: u8, msg: RosflightGnssMsg) {
        self.gnss_count += 1;
        self.last_gnss = Some(msg);
    }

    fn send_cmd_ack(&mut self, _board: &mut TestBoard, _system_id: u8, msg: RosflightCmdAckMsg) {
        self.cmd_ack_count += 1;
        self.last_cmd_ack = Some(msg);
    }

    fn send_rc_channels(&mut self, _board: &mut TestBoard, _system_id: u8, msg: RcChannelsMsg) {
        self.rc_channels_count += 1;
        self.last_rc_channels = Some(msg);
    }

    fn send_battery_status(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        _msg: BatteryStatusMsg,
    ) {
        self.battery_count += 1;
    }

    fn send_statustext(&mut self, _board: &mut TestBoard, _system_id: u8, msg: StatustextMsg) {
        self.statustext_count += 1;
        self.last_statustext = Some(msg);
    }

    fn send_hard_error(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        msg: RosflightHardErrorMsg,
    ) {
        self.hard_error_count += 1;
        self.last_hard_error = Some(msg);
    }

    fn handle_incoming_messages(&mut self, _board: &mut TestBoard, _msgs: &mut Messages) {}
}
