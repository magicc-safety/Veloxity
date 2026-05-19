use crate::board;
use crate::comm::messages::{Messages, messages::*};

pub trait CommInterface<B: board::BoardIo> {
    fn send_heartbeat(&mut self, board: &mut B, system_id: u8, msg: HeartbeatMsg) -> bool;
    fn send_named_value(&mut self, board: &mut B, system_id: u8, msg: ParamValueMsg);
    fn send_status(&mut self, board: &mut B, system_id: u8, msg: RosflightStatusMsg);
    fn send_timesync(&mut self, board: &mut B, system_id: u8, msg: TimesyncMsg) -> bool;
    fn send_version(&mut self, board: &mut B, system_id: u8, msg: RosflightVersionMsg);
    fn send_output_raw(&mut self, baord: &mut B, system_id: u8, msg: RosflightOutputRawMsg);
    fn send_attitude(&mut self, board: &mut B, system_id: u8, msg: AttitudeQuaternionMsg);
    fn send_baro(&mut self, board: &mut B, system_id: u8, msg: SmallBaroMsg);
    fn send_diff_pressure(&mut self, board: &mut B, system_id: u8, msg: DiffPressureMsg);
    fn send_imu(&mut self, board: &mut B, system_id: u8, msg: SmallImuMsg);
    fn send_mag(&mut self, board: &mut B, system_id: u8, msg: SmallMagMsg);
    fn send_rc_raw(&mut self, board: &mut B, system_id: u8, msg: RcChannelsMsg);
    fn send_range(&mut self, board: &mut B, system_id: u8, msg: SmallRangeMsg);
    fn send_gnss(&mut self, board: &mut B, system_id: u8, msg: RosflightGnssMsg);
    fn send_cmd_ack(&mut self, board: &mut B, system_id: u8, msg: RosflightCmdAckMsg);
    fn send_rc_channels(&mut self, board: &mut B, system_id: u8, msg: RcChannelsMsg);
    fn send_battery_status(&mut self, board: &mut B, system_id: u8, msg: BatteryStatusMsg);
    fn send_statustext(&mut self, board: &mut B, system_id: u8, msg: StatustextMsg);
    fn send_hard_error(&mut self, board: &mut B, system_id: u8, msg: RosflightHardErrorMsg);

    fn handle_incoming_messages(&mut self, board: &mut B, msgs: &mut Messages);
}

#[allow(async_fn_in_trait)]
pub trait EmbeddedComInterface {
    async fn process_bytes(&mut self, buf: &[u8], num_bytes: usize);
}
