use crate::{
    board::BoardTrait,
    comm_manager::comm_link_trait::CommInterface,
    comm_messages::{self, messages::*},
    errors,
    hlist::HNil,
};

#[derive(Default)]
pub struct TestBoard {
    pub current_time_us: u64,
    pub tx_write_count: usize,
}

impl BoardTrait for TestBoard {
    type RawSensorSet = HNil;
    type ProcessedSensorSet = HNil;
    type ProcessorHList = HNil;

    fn update_sensors(&mut self, _sensors: &mut Self::RawSensorSet) {}

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
}

pub struct RecordingCommLink {
    pub sent_param_values: [Option<ParamValueMsg>; 8],
    pub sent_param_value_count: usize,
    pub heartbeat_count: usize,
    pub status_count: usize,
    pub last_status: Option<RosflightStatusMsg>,
    pub imu_count: usize,
    pub attitude_count: usize,
    pub output_raw_count: usize,
    pub last_output_raw: Option<RosflightOutputRawMsg>,
    pub version_count: usize,
    pub last_version: Option<RosflightVersionMsg>,
    pub cmd_ack_count: usize,
    pub last_cmd_ack: Option<RosflightCmdAckMsg>,
    pub statustext_count: usize,
    pub last_statustext: Option<StatustextMsg>,
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
            output_raw_count: 0,
            last_output_raw: None,
            version_count: 0,
            last_version: None,
            cmd_ack_count: 0,
            last_cmd_ack: None,
            statustext_count: 0,
            last_statustext: None,
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

    fn send_named_value(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        msg: ParamValueMsg,
    ) {
        self.record_param_value(msg);
    }

    fn send_status(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        msg: RosflightStatusMsg,
    ) {
        self.status_count += 1;
        self.last_status = Some(msg);
    }

    fn send_timesync(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        _msg: TimesyncMsg,
    ) -> bool {
        true
    }

    fn send_version(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        msg: RosflightVersionMsg,
    ) {
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

    fn send_baro(&mut self, _board: &mut TestBoard, _system_id: u8, _msg: SmallBaroMsg) {}

    fn send_diff_pressure(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        _msg: DiffPressureMsg,
    ) {
    }

    fn send_imu(&mut self, _board: &mut TestBoard, _system_id: u8, _msg: SmallImuMsg) {
        self.imu_count += 1;
    }

    fn send_mag(&mut self, _board: &mut TestBoard, _system_id: u8, _msg: SmallMagMsg) {}

    fn send_rc_raw(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        _msg: RosflightOutputRawMsg,
    ) {
    }

    fn send_range(&mut self, _board: &mut TestBoard, _system_id: u8, _msg: SmallRangeMsg) {}

    fn send_gnss(&mut self, _board: &mut TestBoard, _system_id: u8, _msg: RosflightGnssMsg) {}

    fn send_cmd_ack(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        msg: RosflightCmdAckMsg,
    ) {
        self.cmd_ack_count += 1;
        self.last_cmd_ack = Some(msg);
    }

    fn send_rc_channels(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        _msg: RcChannelsMsg,
    ) {
    }

    fn send_battery_status(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        _msg: BatteryStatusMsg,
    ) {
    }

    fn send_statustext(
        &mut self,
        _board: &mut TestBoard,
        _system_id: u8,
        msg: StatustextMsg,
    ) {
        self.statustext_count += 1;
        self.last_statustext = Some(msg);
    }

    fn handle_incoming_messages(
        &mut self,
        _board: &mut TestBoard,
        _msgs: &mut comm_messages::Messages,
    ) {
    }
}
