use crate::comm_manager::{comm_link_trait::CommInterface, mavlink_parser};
use crate::comm_messages::{Messages, Store, enums as comm_enums, messages as comm_messages};
use crate::mavlink::dialects::rosflight::{
    Rosflight, enums as mav_enums, messages as mav_messages,
};
use crate::{board, packets};
use core::result::Result;
use mavio::Frame;
use mavio::prelude::*;
use mavio::protocol::{DialectVersion, FrameBuilder};

static RX_BUFF_SIZE: usize = 2048;
const MAV_COMP_ID_ROSFLIGHT_FIRMWARE: u8 = 250;
const MAVLINK_V1_MESSAGE_SIZE: usize = 263;

pub struct MavlinkInterface {
    pub component_id: u8,
    sequence: u8,
    mav_parser: mavlink_parser::MavlinkParser,
}

impl MavlinkInterface {
    pub fn new() -> Self {
        Self {
            component_id: MAV_COMP_ID_ROSFLIGHT_FIRMWARE, // In latest rosflight_firmware this is hardcoded to 250
            sequence: 0,
            mav_parser: mavlink_parser::MavlinkParser::new(),
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
                msgs.store(comm_messages::ExternalAttitudeMsg::from(es))
            }
            Rosflight::Timesync(ts) => msgs.store(comm_messages::TimesyncMsg::from(ts)),
            Rosflight::RosflightCmd(cmd) => msgs.store(comm_messages::RosflightCmdMsg::from(cmd)),
            Rosflight::RosflightAuxCmd(aux_cmd) => {
                msgs.store(comm_messages::RosflightAuxCmdMsg::from(aux_cmd))
            }
            Rosflight::OffboardControl(oc) => {
                msgs.store(comm_messages::OffboardControlMsg::from(oc))
            }
            Rosflight::ParamRequestRead(pr) => {
                msgs.store(comm_messages::ParamRequestReadMsg::from(pr))
            }
            Rosflight::ParamSet(ps) => msgs.store(comm_messages::ParamSetMsg::from(ps)),
            Rosflight::ParamRequestList(pl) => {
                msgs.store(comm_messages::ParamRequestListMsg::from(pl))
            }
            Rosflight::Heartbeat(hb) => msgs.store(comm_messages::HeartbeatMsg::from(hb)),
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
                        if let Some(message) = mavlink_parser::process_mavlink_frame(frame) {
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
        msg: comm_messages::RosflightStatusMsg,
    ) {
        self.send_message(board, system_id, mav_messages::RosflightStatus::from(msg));
    }
    fn send_timesync(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::TimesyncMsg,
    ) -> bool {
        self.send_message(board, system_id, mav_messages::Timesync::from(msg));
        return true;
    }
    fn send_named_value(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::ParamValueMsg,
    ) {
        self.send_message(board, system_id, mav_messages::ParamValue::from(msg));
    }
    fn send_heartbeat(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::HeartbeatMsg,
    ) -> bool {
        self.send_message(board, system_id, mav_messages::Heartbeat::from(msg));
        return true;
    }
    fn send_version(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::RosflightVersionMsg,
    ) {
        self.send_message(board, system_id, mav_messages::RosflightVersion::from(msg));
    }
    fn send_diff_pressure(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::DiffPressureMsg,
    ) {
        self.send_message(board, system_id, mav_messages::DiffPressure::from(msg));
    }
    fn send_baro(&mut self, board: &mut B, system_id: u8, msg: comm_messages::SmallBaroMsg) {
        self.send_message(board, system_id, mav_messages::SmallBaro::from(msg));
    }
    fn send_imu(&mut self, board: &mut B, system_id: u8, msg: comm_messages::SmallImuMsg) {
        self.send_message(board, system_id, mav_messages::SmallImu::from(msg));
    }
    fn send_attitude(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::AttitudeQuaternionMsg,
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
        msg: comm_messages::RosflightOutputRawMsg,
    ) {
        self.send_message(
            board,
            system_id,
            mav_messages::RosflightOutputRaw::from(msg),
        );
    }
    fn send_rc_raw(&mut self, board: &mut B, system_id: u8, msg: comm_messages::RcChannelsMsg) {
        self.send_message(board, system_id, mav_messages::RcChannels::from(msg));
    }
    fn send_range(&mut self, board: &mut B, system_id: u8, msg: comm_messages::SmallRangeMsg) {
        self.send_message(board, system_id, mav_messages::SmallRange::from(msg));
    }
    fn send_mag(&mut self, board: &mut B, system_id: u8, msg: comm_messages::SmallMagMsg) {
        self.send_message(board, system_id, mav_messages::SmallMag::from(msg));
    }
    fn send_gnss(&mut self, board: &mut B, system_id: u8, msg: comm_messages::RosflightGnssMsg) {
        self.send_message(board, system_id, mav_messages::RosflightGnss::from(msg));
    }
    fn send_cmd_ack(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::RosflightCmdAckMsg,
    ) {
        self.send_message(board, system_id, mav_messages::RosflightCmdAck::from(msg));
    }
    fn send_rc_channels(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::RcChannelsMsg,
    ) {
        self.send_message(board, system_id, mav_messages::RcChannels::from(msg));
    }
    fn send_battery_status(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::BatteryStatusMsg,
    ) {
        self.send_message(
            board,
            system_id,
            mav_messages::RosflightBatteryStatus::from(msg),
        );
    }
    fn send_statustext(&mut self, board: &mut B, system_id: u8, msg: comm_messages::StatustextMsg) {
        self.send_message(board, system_id, mav_messages::Statustext::from(msg));
    }
    fn send_hard_error(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: comm_messages::RosflightHardErrorMsg,
    ) {
        self.send_message(
            board,
            system_id,
            mav_messages::RosflightHardError::from(msg),
        );
    }
}

impl From<mav_enums::RosflightAuxCmdType> for comm_enums::RosflightAuxCmdType {
    fn from(val: mav_enums::RosflightAuxCmdType) -> Self {
        use comm_enums::RosflightAuxCmdType as CommCmd;
        use mav_enums::RosflightAuxCmdType as MavCmd;
        match val {
            MavCmd::Disabled => CommCmd::Disabled,
            MavCmd::Motor => CommCmd::Motor,
            MavCmd::Servo => CommCmd::Servo,
        }
    }
}

// RECEIVING MSG CONVERSIONS
impl From<mav_messages::RosflightCmd> for comm_messages::RosflightCmdMsg {
    fn from(msg: mav_messages::RosflightCmd) -> Self {
        Self {
            command: comm_enums::RosflightCmd::from(msg.command),
        }
    }
}

impl From<mav_messages::Timesync> for comm_messages::TimesyncMsg {
    fn from(msg: mav_messages::Timesync) -> Self {
        Self {
            tc1: msg.tc1,
            ts1: msg.ts1,
        }
    }
}

impl From<mav_messages::ExternalAttitude> for comm_messages::ExternalAttitudeMsg {
    fn from(msg: mav_messages::ExternalAttitude) -> Self {
        Self {
            qx: msg.qx,
            qw: msg.qw,
            qy: msg.qy,
            qz: msg.qz,
        }
    }
}

impl From<mav_messages::OffboardControl> for comm_messages::OffboardControlMsg {
    fn from(msg: mav_messages::OffboardControl) -> Self {
        use comm_enums::OffboardControlIgnore as CommIgnore;
        use comm_enums::OffboardControlMode as CommMode;
        use mav_enums::OffboardControlIgnore as MavIgnore;
        use mav_enums::OffboardControlMode as MavMode;

        // --- Build the new `ignore` flags by checking the bits ---
        let mut comm_ignore = CommIgnore::empty(); // Start with no flags set

        let ignore_mask = msg.ignore as u16;

        if (ignore_mask & (MavIgnore::IgnoreValue0 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_FX;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue1 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_FY;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue2 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_FZ;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue3 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_QX;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue4 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_QY;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue5 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_QZ;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue6 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_PASS_0;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue7 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_PASS_1;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue8 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_PASS_2;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue9 as u16)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_PASS_3;
        }

        Self {
            mode: match msg.mode {
                MavMode::ModePassThrough => CommMode::ModePassThrough,
                MavMode::ModeRollratePitchrateYawrateThrottle => {
                    CommMode::ModeRollratePitchrateYawrateThrottle
                }
                MavMode::ModeRollPitchYawrateThrottle => CommMode::ModeRollPitchYawrateThrottle,
                _ => CommMode::ModePassThrough, // Default for other modes
            },
            ignore: comm_ignore, // Use the flags we just built
            qx: msg.u[3],
            qy: msg.u[4],
            qz: msg.u[5],
            fx: msg.u[0],
            fy: msg.u[1],
            fz: msg.u[2],
            passthrough: [msg.u[6], msg.u[7], msg.u[8], msg.u[9]],
        }
    }
}

impl From<mav_messages::RosflightAuxCmd> for comm_messages::RosflightAuxCmdMsg {
    fn from(msg: mav_messages::RosflightAuxCmd) -> Self {
        Self {
            type_array: msg.type_array.map(|t| t.into()),
            aux_cmd_array: msg.aux_cmd_array,
        }
    }
}

impl From<mav_messages::Heartbeat> for comm_messages::HeartbeatMsg {
    fn from(msg: mav_messages::Heartbeat) -> Self {
        Self {
            type_: msg.type_,
            autopilot: msg.autopilot,
            base_mode: msg.base_mode,
            custom_mode: msg.custom_mode,
            system_status: msg.system_status,
            mavlink_version: msg.mavlink_version,
        }
    }
}

impl From<mav_messages::ParamRequestRead> for comm_messages::ParamRequestReadMsg {
    fn from(msg: mav_messages::ParamRequestRead) -> Self {
        Self {
            target_system: msg.target_system,
            target_component: msg.target_component,
            /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
            param_identifier: match msg.param_index {
                -1 => comm_enums::ParamIdentifier::ID(msg.param_id),
                _ => comm_enums::ParamIdentifier::INDEX(msg.param_index),
            },
        }
    }
}

impl From<mav_messages::ParamRequestList> for comm_messages::ParamRequestListMsg {
    fn from(msg: mav_messages::ParamRequestList) -> Self {
        Self {
            target_system: msg.target_system,
            target_component: msg.target_component,
        }
    }
}

impl From<mav_messages::ParamSet> for comm_messages::ParamSetMsg {
    fn from(msg: mav_messages::ParamSet) -> Self {
        use crate::params::ParamValue;
        use mav_enums::MavParamType::*;
        Self {
            target_system: msg.target_system,
            target_component: msg.target_component,
            param_id: msg.param_id,
            param_value: match msg.param_type {
                Uint8 | Uint16 | Uint32 | Uint64 => {
                    ParamValue::Uint(f32::to_bits(msg.param_value) as u32)
                }
                Int8 | Int16 | Int32 | Int64 => {
                    ParamValue::Int(f32::to_bits(msg.param_value) as i32)
                }
                Real32 | Real64 => ParamValue::Float(msg.param_value),
            },
        }
    }
}

// SENDING MSG CONVERSIONS

impl From<comm_messages::TimesyncMsg> for mav_messages::Timesync {
    fn from(msg: comm_messages::TimesyncMsg) -> Self {
        Self {
            tc1: msg.tc1,
            ts1: msg.ts1,
        }
    }
}

impl From<comm_messages::RosflightStatusMsg> for mav_messages::RosflightStatus {
    fn from(msg: comm_messages::RosflightStatusMsg) -> Self {
        Self {
            armed: msg.armed,
            failsafe: msg.failsafe,
            rc_override: msg.rc_override,
            offboard: msg.offboard,
            error_code: mav_enums::RosflightErrorCode::from_bits_truncate(
                msg.error_code.bits() as u8
            ),
            control_mode: msg.control_mode.into(),
            num_errors: msg.num_errors,
            loop_time_us: msg.loop_time_us,
        }
    }
}

impl From<comm_enums::OffboardControlMode> for mav_enums::OffboardControlMode {
    fn from(val: comm_enums::OffboardControlMode) -> Self {
        use comm_enums::OffboardControlMode as CommMode;
        use mav_enums::OffboardControlMode as MavMode;
        match val {
            CommMode::ModePassThrough => MavMode::ModePassThrough,
            CommMode::ModeRollratePitchrateYawrateThrottle => {
                MavMode::ModeRollratePitchrateYawrateThrottle
            }
            CommMode::ModeRollPitchYawrateThrottle => MavMode::ModeRollPitchYawrateThrottle,
        }
    }
}

impl From<comm_messages::RosflightVersionMsg> for mav_messages::RosflightVersion {
    fn from(msg: comm_messages::RosflightVersionMsg) -> Self {
        Self {
            version: msg.version,
        }
    }
}

impl From<comm_messages::SmallImuMsg> for mav_messages::SmallImu {
    fn from(msg: comm_messages::SmallImuMsg) -> Self {
        Self {
            time_boot_us: msg.time_boot_us,
            xacc: msg.xacc,
            yacc: msg.yacc,
            zacc: msg.zacc,
            xgyro: msg.xgyro,
            ygyro: msg.ygyro,
            zgyro: msg.zgyro,
            temperature: msg.temperature,
        }
    }
}

impl From<comm_enums::RosflightRangeType> for mav_enums::RosflightRangeType {
    fn from(val: comm_enums::RosflightRangeType) -> Self {
        use comm_enums::RosflightRangeType as CommType;
        use mav_enums::RosflightRangeType as MavType;
        match val {
            CommType::RosflightRangeSonar => MavType::RosflightRangeSonar,
            CommType::RosflightRangeLidar => MavType::RosflightRangeLidar,
        }
    }
}

impl From<comm_messages::SmallRangeMsg> for mav_messages::SmallRange {
    fn from(msg: comm_messages::SmallRangeMsg) -> Self {
        Self {
            type_: msg.type_.into(),
            range: msg.range,
            max_range: msg.max_range,
            min_range: msg.min_range,
        }
    }
}

impl From<comm_messages::SmallMagMsg> for mav_messages::SmallMag {
    fn from(msg: comm_messages::SmallMagMsg) -> Self {
        Self {
            xmag: msg.xmag,
            ymag: msg.ymag,
            zmag: msg.zmag,
        }
    }
}

impl From<comm_messages::SmallBaroMsg> for mav_messages::SmallBaro {
    fn from(msg: comm_messages::SmallBaroMsg) -> Self {
        Self {
            altitude: msg.altitude,
            pressure: msg.pressure,
            temperature: msg.temperature,
        }
    }
}

impl From<comm_messages::HeartbeatMsg> for mav_messages::Heartbeat {
    fn from(msg: comm_messages::HeartbeatMsg) -> Self {
        Self {
            type_: msg.type_,
            autopilot: msg.autopilot,
            base_mode: msg.base_mode,
            custom_mode: msg.custom_mode,
            system_status: msg.system_status,
            mavlink_version: msg.mavlink_version,
        }
    }
}

impl From<comm_messages::DiffPressureMsg> for mav_messages::DiffPressure {
    fn from(msg: comm_messages::DiffPressureMsg) -> Self {
        Self {
            velocity: msg.velocity,
            diff_pressure: msg.diff_pressure,
            temperature: msg.temperature,
        }
    }
}

impl From<comm_messages::AttitudeQuaternionMsg> for mav_messages::AttitudeQuaternion {
    fn from(msg: comm_messages::AttitudeQuaternionMsg) -> Self {
        Self {
            time_boot_ms: msg.time_boot_ms,
            q1: msg.q1,
            q2: msg.q2,
            q3: msg.q3,
            q4: msg.q4,
            rollspeed: msg.rollspeed,
            pitchspeed: msg.pitchspeed,
            yawspeed: msg.yawspeed,
        }
    }
}

impl From<comm_messages::RosflightOutputRawMsg> for mav_messages::RosflightOutputRaw {
    fn from(msg: comm_messages::RosflightOutputRawMsg) -> Self {
        Self {
            stamp: msg.stamp,
            values: msg.values,
        }
    }
}

impl From<comm_enums::GnssFixType> for mav_enums::GnssFixType {
    fn from(val: comm_enums::GnssFixType) -> Self {
        use comm_enums::GnssFixType as CommType;
        use mav_enums::GnssFixType as MavType;
        match val {
            CommType::GnssFixNoFix => MavType::GnssFixNoFix,
            CommType::GnssFixDeadReckoningOnly => MavType::GnssFixDeadReckoningOnly,
            CommType::GnssFix2dFix => MavType::GnssFix2dFix,
            CommType::GnssFix3dFix => MavType::GnssFix3dFix,
            CommType::GnssFixGnssPlusDeadReckoning => MavType::GnssFixGnssPlusDeadReckoning,
            CommType::GnssFixTimeFixOnly => MavType::GnssFixTimeFixOnly,
        }
    }
}

impl From<comm_messages::RosflightGnssMsg> for mav_messages::RosflightGnss {
    fn from(msg: comm_messages::RosflightGnssMsg) -> Self {
        Self {
            seconds: msg.seconds,
            nanos: msg.nanos,
            fix_type: msg.fix_type.into(), // This uses your From impl at line 548
            num_sat: msg.num_sat,
            lat: msg.lat,
            lon: msg.lon,
            height: msg.height,
            vel_n: msg.vel_n,
            vel_e: msg.vel_e,
            vel_d: msg.vel_d,
            h_acc: msg.h_acc,
            v_acc: msg.v_acc,
            s_acc: msg.s_acc,
            rosflight_timestamp: msg.rosflight_timestamp,
        }
    }
}

impl From<comm_messages::ParamValueMsg> for mav_messages::ParamValue {
    fn from(msg: comm_messages::ParamValueMsg) -> Self {
        use crate::params::ParamValue as CommParamValue;
        use mav_enums::MavParamType;
        let (value_f32, value_type) = match msg.param_value {
            // Should the ParamValue type be updated to support smaller types?
            CommParamValue::Float(f) => (f, MavParamType::Real32),
            CommParamValue::Int(i) => (f32::from_bits(i as u32), MavParamType::Int32),
            CommParamValue::Uint(u) => (f32::from_bits(u), MavParamType::Uint32),
            CommParamValue::Bool(b) => {
                (f32::from_bits(if b { 1 } else { 0 }), MavParamType::Uint32)
            } // Not sure if it's okay to pass these as floats if raw byte value is being used
        };
        Self {
            param_id: msg.param_id,
            param_value: value_f32,
            param_type: value_type,
            param_count: msg.param_count,
            param_index: msg.param_index,
        }
    }
}

impl From<comm_messages::RosflightCmdAckMsg> for mav_messages::RosflightCmdAck {
    fn from(msg: comm_messages::RosflightCmdAckMsg) -> Self {
        Self {
            // Convert the nested enums using their respective From impls
            command: msg.command.into(),
            success: msg.success.into(),
        }
    }
}

impl From<comm_enums::RosflightCmd> for mav_enums::RosflightCmd {
    fn from(val: comm_enums::RosflightCmd) -> Self {
        use comm_enums::RosflightCmd as CommCmd;
        use mav_enums::RosflightCmd as MavCmd;
        match val {
            CommCmd::RcCalibration => MavCmd::RcCalibration,
            CommCmd::AccelCalibration => MavCmd::AccelCalibration,
            CommCmd::GyroCalibration => MavCmd::GyroCalibration,
            CommCmd::BaroCalibration => MavCmd::BaroCalibration,
            CommCmd::AirspeedCalibration => MavCmd::AirspeedCalibration,
            CommCmd::ReadParams => MavCmd::ReadParams,
            CommCmd::WriteParams => MavCmd::WriteParams,
            CommCmd::SetParamDefaults => MavCmd::SetParamDefaults,
            CommCmd::Reboot => MavCmd::Reboot,
            CommCmd::RebootToBootloader => MavCmd::RebootToBootloader,
            CommCmd::SendVersion => MavCmd::SendVersion,
            CommCmd::ResetOrigin => MavCmd::ResetOrigin,
            CommCmd::SendAllConfigInfos => MavCmd::SendAllConfigInfos,
        }
    }
}

impl From<mav_enums::RosflightCmd> for comm_enums::RosflightCmd {
    fn from(val: mav_enums::RosflightCmd) -> Self {
        use comm_enums::RosflightCmd as CommCmd;
        use mav_enums::RosflightCmd as MavCmd;
        match val {
            MavCmd::RcCalibration => CommCmd::RcCalibration,
            MavCmd::AccelCalibration => CommCmd::AccelCalibration,
            MavCmd::GyroCalibration => CommCmd::GyroCalibration,
            MavCmd::BaroCalibration => CommCmd::BaroCalibration,
            MavCmd::AirspeedCalibration => CommCmd::AirspeedCalibration,
            MavCmd::ReadParams => CommCmd::ReadParams,
            MavCmd::WriteParams => CommCmd::WriteParams,
            MavCmd::SetParamDefaults => CommCmd::SetParamDefaults,
            MavCmd::Reboot => CommCmd::Reboot,
            MavCmd::RebootToBootloader => CommCmd::RebootToBootloader,
            MavCmd::SendVersion => CommCmd::SendVersion,
            MavCmd::ResetOrigin => CommCmd::ResetOrigin,
            MavCmd::SendAllConfigInfos => CommCmd::SendAllConfigInfos,
        }
    }
}

impl From<comm_enums::RosflightCmdResponse> for mav_enums::RosflightCmdResponse {
    fn from(val: comm_enums::RosflightCmdResponse) -> Self {
        use comm_enums::RosflightCmdResponse as CommCmd;
        use mav_enums::RosflightCmdResponse as MavCmd;
        match val {
            CommCmd::RosflightCmdFailed => MavCmd::RosflightCmdFailed,
            CommCmd::RosflightCmdSuccess => MavCmd::RosflightCmdSuccess,
        }
    }
}

impl From<mav_enums::RosflightCmdResponse> for comm_enums::RosflightCmdResponse {
    fn from(val: mav_enums::RosflightCmdResponse) -> Self {
        use comm_enums::RosflightCmdResponse as CommResponse;
        use mav_enums::RosflightCmdResponse as MavResponse;
        match val {
            MavResponse::RosflightCmdFailed => CommResponse::RosflightCmdFailed,
            MavResponse::RosflightCmdSuccess => CommResponse::RosflightCmdSuccess,
        }
    }
}

impl From<comm_messages::RcChannelsMsg> for mav_messages::RcChannels {
    fn from(msg: comm_messages::RcChannelsMsg) -> Self {
        // `msg.channels` is [u16; 16] (from RC_PACKET_CHANNELS)
        // `Self` (RcChannels) has 18 individual channel fields.

        Self {
            time_boot_ms: msg.time_boot_ms,
            chancount: msg.chancount,
            rssi: msg.rssi,

            // Map the array to the individual fields
            chan1_raw: msg.channels[0],
            chan2_raw: msg.channels[1],
            chan3_raw: msg.channels[2],
            chan4_raw: msg.channels[3],
            chan5_raw: msg.channels[4],
            chan6_raw: msg.channels[5],
            chan7_raw: msg.channels[6],
            chan8_raw: msg.channels[7],
            chan9_raw: msg.channels[8],
            chan10_raw: msg.channels[9],
            chan11_raw: msg.channels[10],
            chan12_raw: msg.channels[11],
            chan13_raw: msg.channels[12],
            chan14_raw: msg.channels[13],
            chan15_raw: msg.channels[14],
            chan16_raw: msg.channels[15],

            // Set remaining MAVLink channels to "unused"
            chan17_raw: u16::MAX,
            chan18_raw: u16::MAX,
        }
    }
}

impl From<comm_messages::BatteryStatusMsg> for mav_messages::RosflightBatteryStatus {
    fn from(msg: comm_messages::BatteryStatusMsg) -> Self {
        Self {
            battery_voltage: msg.battery_voltage,
            battery_current: msg.battery_current,
        }
    }
}

impl From<packets::GNSSFixType> for mav_enums::GnssFixType {
    fn from(val: packets::GNSSFixType) -> Self {
        use mav_enums::GnssFixType as MavType;
        use packets::GNSSFixType as CommType;
        match val {
            CommType::NoFix => MavType::GnssFixNoFix,
            CommType::DeadReckoningOnly => MavType::GnssFixDeadReckoningOnly,
            CommType::TwoD => MavType::GnssFix2dFix,
            CommType::ThreeD => MavType::GnssFix3dFix,
            CommType::GnssPlusDeadReckoning => MavType::GnssFixGnssPlusDeadReckoning,
            CommType::TimeFixOnly => MavType::GnssFixTimeFixOnly,
        }
    }
}

impl From<comm_messages::StatustextMsg> for mav_messages::Statustext {
    fn from(msg: comm_messages::StatustextMsg) -> Self {
        Self {
            severity: msg.severity.into(),
            text: msg.text,
        }
    }
}

impl From<comm_messages::RosflightHardErrorMsg> for mav_messages::RosflightHardError {
    fn from(msg: comm_messages::RosflightHardErrorMsg) -> Self {
        Self {
            error_code: msg.error_code,
            pc: msg.pc,
            reset_count: msg.reset_count,
            do_rearm: msg.do_rearm,
        }
    }
}

impl From<comm_enums::Severity> for mav_enums::MavSeverity {
    fn from(val: comm_enums::Severity) -> Self {
        use comm_enums::Severity;
        use mav_enums::MavSeverity;
        match val {
            Severity::Emergency => MavSeverity::Emergency,
            Severity::Alert => MavSeverity::Alert,
            Severity::Critical => MavSeverity::Critical,
            Severity::Error => MavSeverity::Error,
            Severity::Warning => MavSeverity::Warning,
            Severity::Notice => MavSeverity::Notice,
            Severity::Info => MavSeverity::Info,
            Severity::Debug => MavSeverity::Debug,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offboard_control_conversion_preserves_roll_pitch_mode() {
        let msg = mav_messages::OffboardControl {
            mode: mav_enums::OffboardControlMode::ModeRollPitchYawrateThrottle,
            ignore: mav_enums::OffboardControlIgnore::IgnoreNone,
            u: [0.4, 0.5, 0.6, 0.1, 0.2, 0.3, 0.7, 0.8, 0.9, 1.0],
        };

        let converted = comm_messages::OffboardControlMsg::from(msg);

        assert_eq!(
            converted.mode,
            comm_enums::OffboardControlMode::ModeRollPitchYawrateThrottle
        );
        assert!(converted.ignore.is_empty());
        assert_eq!(converted.qx, 0.1);
        assert_eq!(converted.fz, 0.6);
        assert_eq!(converted.passthrough, [0.7, 0.8, 0.9, 1.0]);
    }

    #[test]
    fn offboard_control_conversion_maps_ignore_bits_one_to_one() {
        let mappings = [
            (
                mav_enums::OffboardControlIgnore::IgnoreValue0,
                comm_enums::OffboardControlIgnore::IGNORE_FX,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue1,
                comm_enums::OffboardControlIgnore::IGNORE_FY,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue2,
                comm_enums::OffboardControlIgnore::IGNORE_FZ,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue3,
                comm_enums::OffboardControlIgnore::IGNORE_QX,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue4,
                comm_enums::OffboardControlIgnore::IGNORE_QY,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue5,
                comm_enums::OffboardControlIgnore::IGNORE_QZ,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue6,
                comm_enums::OffboardControlIgnore::IGNORE_PASS_0,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue7,
                comm_enums::OffboardControlIgnore::IGNORE_PASS_1,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue8,
                comm_enums::OffboardControlIgnore::IGNORE_PASS_2,
            ),
            (
                mav_enums::OffboardControlIgnore::IgnoreValue9,
                comm_enums::OffboardControlIgnore::IGNORE_PASS_3,
            ),
        ];

        for (mav_ignore, expected_comm_ignore) in mappings {
            let msg = mav_messages::OffboardControl {
                mode: mav_enums::OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: mav_ignore,
                u: [0.0; 10],
            };

            let converted = comm_messages::OffboardControlMsg::from(msg);

            assert_eq!(converted.ignore, expected_comm_ignore);
        }
    }

    #[test]
    fn offboard_control_conversion_preserves_auxiliary_values_and_ignore_bits() {
        let msg = mav_messages::OffboardControl {
            mode: mav_enums::OffboardControlMode::ModePassThrough,
            ignore: mav_enums::OffboardControlIgnore::IgnoreValue9,
            u: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.7, 0.8, 0.9],
        };

        let converted = comm_messages::OffboardControlMsg::from(msg);

        assert_eq!(converted.passthrough, [0.6, 0.7, 0.8, 0.9]);
        assert_eq!(
            converted.ignore,
            comm_enums::OffboardControlIgnore::IGNORE_PASS_3
        );
    }
}
