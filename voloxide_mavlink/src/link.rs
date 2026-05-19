use crate::generated::dialects::rosflight::{Rosflight, messages as mav_messages};
use crate::parser;
use core::result::Result;
use mavio::Frame;
use mavio::prelude::*;
use mavio::protocol::{DialectVersion, FrameBuilder};
use voloxide_core::board;
use voloxide_core::comm::interface::CommInterface;
use voloxide_core::comm::messages::{Messages, Store, messages as core_messages};

static RX_BUFF_SIZE: usize = 2048;
const MAV_COMP_ID_ROSFLIGHT_FIRMWARE: u8 = 250;
const MAVLINK_V1_MESSAGE_SIZE: usize = 263;

pub struct MavlinkInterface {
    pub component_id: u8,
    sequence: u8,
    mav_parser: parser::MavlinkParser,
}

impl MavlinkInterface {
    pub fn new() -> Self {
        Self {
            component_id: MAV_COMP_ID_ROSFLIGHT_FIRMWARE, // In latest rosflight_firmware this is hardcoded to 250
            sequence: 0,
            mav_parser: parser::MavlinkParser::new(),
        }
    }

    fn frame_builder<T: Message>(&mut self, system_id: u8, msg: T) -> mavio::Result<Frame<V1>> {
        let frame = Frame::builder()
            .version(V1)
            .system_id(system_id)
            .component_id(self.component_id)
            .sequence(self.sequence)
            .message(&msg)?
            .build();

        // Increment the sequence number, wrapping on overflow
        self.sequence = self.sequence.wrapping_add(1);

        Ok(frame)
    }

    fn send_message<B: board::BoardIo, T: Message>(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: T,
    ) {
        let frame = match self.frame_builder(system_id, msg) {
            Ok(f) => f,
            Err(_) => {
                return;
            }
        };

        // MAVLink V1 specification shows messages max size of 263 bytes
        let mut buf = [0u8; MAVLINK_V1_MESSAGE_SIZE];

        if frame.body_length() > buf.len() {
            return;
        }

        let mut pos = 0;
        let header = frame.header();
        let payload = frame.payload().bytes();
        let crc = frame.checksum();

        // MAVLink v1 wire format
        buf[pos] = 0xFE;
        pos += 1; // Start marker
        buf[pos] = payload.len() as u8;
        pos += 1; // Payload length
        buf[pos] = header.sequence();
        pos += 1; // Sequence
        buf[pos] = header.system_id();
        pos += 1; // System ID
        buf[pos] = header.component_id();
        pos += 1; // Component ID
        buf[pos] = header.message_id() as u8;
        pos += 1; // Message ID (low byte)

        // Copy payload
        buf[pos..pos + payload.len()].copy_from_slice(payload);
        pos += payload.len();

        // CRC (little-endian)
        buf[pos..pos + 2].copy_from_slice(&crc.to_le_bytes());
        pos += 2;

        board.serial_tx_write(&buf[..pos]);
    }

    fn process_rosflight_message(&mut self, message: Rosflight, msgs: &mut Messages) {
        match (message) {
            Rosflight::ExternalAttitude(es) => {
                msgs.store(core_messages::ExternalAttitudeMsg::from(es))
            }
            Rosflight::Timesync(ts) => msgs.store(core_messages::TimesyncMsg::from(ts)),
            Rosflight::RosflightCmd(cmd) => msgs.store(core_messages::RosflightCmdMsg::from(cmd)),
            Rosflight::RosflightAuxCmd(aux_cmd) => {
                msgs.store(core_messages::RosflightAuxCmdMsg::from(aux_cmd))
            }
            Rosflight::OffboardControl(oc) => {
                msgs.store(core_messages::OffboardControlMsg::from(oc))
            }
            Rosflight::ParamRequestRead(pr) => {
                msgs.store(core_messages::ParamRequestReadMsg::from(pr))
            }
            Rosflight::ParamSet(ps) => msgs.store(core_messages::ParamSetMsg::from(ps)),
            Rosflight::ParamRequestList(pl) => {
                msgs.store(core_messages::ParamRequestListMsg::from(pl))
            }
            Rosflight::Heartbeat(hb) => msgs.store(core_messages::HeartbeatMsg::from(hb)),
            _ => {}
        }
    }
}

impl<B: board::BoardIo> CommInterface<B> for MavlinkInterface {
    fn handle_incoming_messages(&mut self, board: &mut B, msgs: &mut Messages) {
        let mut buf = [0u8; RX_BUFF_SIZE];
        match board.serial_rx_read(&mut buf) {
            Some(Ok(n)) => {
                for i in 0..n {
                    if let Some(frame) = self.mav_parser.feed_byte(buf[i]) {
                        if let Some(message) = parser::process_mavlink_frame(frame) {
                            self.process_rosflight_message(message, msgs);
                        }
                    }
                }
            }
            Some(Err(_)) => {}
            None => {}
        }
    }

    fn send_status(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightStatusMsg,
    ) {
        self.send_message(board, system_id, mav_messages::RosflightStatus::from(msg));
    }
    fn send_timesync(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::TimesyncMsg,
    ) -> bool {
        self.send_message(board, system_id, mav_messages::Timesync::from(msg));
        return true;
    }
    fn send_named_value(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::ParamValueMsg,
    ) {
        self.send_message(board, system_id, mav_messages::ParamValue::from(msg));
    }
    fn send_heartbeat(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::HeartbeatMsg,
    ) -> bool {
        self.send_message(board, system_id, mav_messages::Heartbeat::from(msg));
        return true;
    }
    fn send_version(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightVersionMsg,
    ) {
        self.send_message(board, system_id, mav_messages::RosflightVersion::from(msg));
    }
    fn send_diff_pressure(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::DiffPressureMsg,
    ) {
        self.send_message(board, system_id, mav_messages::DiffPressure::from(msg));
    }
    fn send_baro(&mut self, board: &mut B, system_id: u8, msg: core_messages::SmallBaroMsg) {
        self.send_message(board, system_id, mav_messages::SmallBaro::from(msg));
    }
    fn send_imu(&mut self, board: &mut B, system_id: u8, msg: core_messages::SmallImuMsg) {
        self.send_message(board, system_id, mav_messages::SmallImu::from(msg));
    }
    fn send_attitude(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::AttitudeQuaternionMsg,
    ) {
        self.send_message(
            board,
            system_id,
            mav_messages::AttitudeQuaternion::from(msg),
        );
    }
    fn send_output_raw(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightOutputRawMsg,
    ) {
        self.send_message(
            board,
            system_id,
            mav_messages::RosflightOutputRaw::from(msg),
        );
    }
    fn send_rc_raw(&mut self, board: &mut B, system_id: u8, msg: core_messages::RcChannelsMsg) {
        self.send_message(board, system_id, mav_messages::RcChannels::from(msg));
    }
    fn send_range(&mut self, board: &mut B, system_id: u8, msg: core_messages::SmallRangeMsg) {
        self.send_message(board, system_id, mav_messages::SmallRange::from(msg));
    }
    fn send_mag(&mut self, board: &mut B, system_id: u8, msg: core_messages::SmallMagMsg) {
        self.send_message(board, system_id, mav_messages::SmallMag::from(msg));
    }
    fn send_gnss(&mut self, board: &mut B, system_id: u8, msg: core_messages::RosflightGnssMsg) {
        self.send_message(board, system_id, mav_messages::RosflightGnss::from(msg));
    }
    fn send_cmd_ack(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightCmdAckMsg,
    ) {
        self.send_message(board, system_id, mav_messages::RosflightCmdAck::from(msg));
    }
    fn send_rc_channels(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RcChannelsMsg,
    ) {
        self.send_message(board, system_id, mav_messages::RcChannels::from(msg));
    }
    fn send_battery_status(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::BatteryStatusMsg,
    ) {
        self.send_message(
            board,
            system_id,
            mav_messages::RosflightBatteryStatus::from(msg),
        );
    }
    fn send_statustext(&mut self, board: &mut B, system_id: u8, msg: core_messages::StatustextMsg) {
        self.send_message(board, system_id, mav_messages::Statustext::from(msg));
    }
    fn send_hard_error(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightHardErrorMsg,
    ) {
        self.send_message(
            board,
            system_id,
            mav_messages::RosflightHardError::from(msg),
        );
    }
}
