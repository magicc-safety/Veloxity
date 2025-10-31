// /**
// ******************************************************************************
// * File     : mavlink.rs
// * Date     : May 8, 2025
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************
// **/
// use defmt;
use crate::{board, packets};
use crate::comm_manager::{comm_link_trait::CommInterface, mavlink_parser};
use crate::comm_messages::{Messages, Store, messages::*, enums as comm_enums};
use crate::mavlink::dialects::rosflight::{Rosflight, messages, messages::*, enums};
use mavio::protocol::{DialectVersion, FrameBuilder};
use mavio::Frame;
use mavio::prelude::*;
use core::result::Result;

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

    fn send_message<B: board::BoardTrait, T: Message>(&mut self, board: &mut B, system_id: u8, msg: T) {
        let frame = match self.frame_builder(system_id, msg) {
            Ok(f) => f,
            Err(_) => {
                // defmt::debug!("Error with FrameBuilder!");
                return;
            }
        };

        // MAVLink V1 specification shows messages max size of 263 bytes
        let mut buf = [0u8; MAVLINK_V1_MESSAGE_SIZE]; 

        if frame.body_length() > buf.len() {
            // defmt::debug!("Body Length Error");
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

    fn process_rosflight_message(&mut self, message: Rosflight, msgs: &mut Messages){
        match (message) {
            Rosflight::ExternalAttitude(es) => {
                // defmt::debug!("External attitude: qw={}, qx={}, qy={}, qz={}", es.qw, es.qx, es.qy, es.qz);
                // println!("Message: ExternalAttitude");
                msgs.store(ExternalAttitudeMsg::from(es))
            }
            Rosflight::Timesync(ts) => {
                // println!("Message: Timesync");
                // defmt::debug!("Timesync: tc1={} ts1={} ", ts.tc1, ts.ts1);
                msgs.store(TimesyncMsg::from(ts))
            }
            Rosflight::RosflightCmd(cmd) => {
                println!("Message: RosflightCmd");
                println!("Rosflight command: {}", cmd.command as u8);
                msgs.store(RosflightCmdMsg::from(cmd))
            }
            Rosflight::RosflightAuxCmd(aux_cmd) => {
                // println!("Message: RosflightAuxCmd");
                // defmt::debug!("Rosflight aux command: type={}, aux_cmd={}", aux_cmd.type_array.map(|t| t as u8), aux_cmd.aux_cmd_array);
                msgs.store(RosflightAuxCmdMsg::from(aux_cmd))
            }
            Rosflight::OffboardControl(oc) => {
                // println!("Message: OffboardControl");
                // defmt::debug!("Offboard control: mode={}, ignore={}, qx={}, qy={}, qz={}", oc.mode as u8, oc.ignore as u8, oc.qx, oc.qy, oc.qz);
                msgs.store(OffboardControlMsg::from(oc))
            }
            Rosflight::ParamRequestRead(pr) => {
                // println!("Message: ParamRequestRead");
                msgs.store(ParamRequestReadMsg::from(pr))
            }
            Rosflight::ParamSet(ps) => {
                // println!("Message: ParamSet");
                msgs.store(ParamSetMsg::from(ps))
            }
            Rosflight::ParamRequestList(pl) => {
                // println!("Message: ParamRequestList");
                msgs.store(ParamRequestListMsg::from(pl))
            }
            Rosflight::Heartbeat(hb) => {
                // defmt::debug!(
                //     "🎉 Heartbeat: autopilot={}, mode={}, status={}, custom_mode: {}",
                //     hb.autopilot,
                //     hb.base_mode,
                //     hb.system_status,
                //     hb.custom_mode,
                // );
                // println!("Message: Heartbeat");
                msgs.store(HeartbeatMsg::from(hb))
            }
            _ => {
                // defmt::debug!("System: Other ROSflight message received");
            }
        }
    }
}

impl<B: board::BoardTrait> CommInterface<B> for MavlinkInterface {
    fn handle_incoming_messages(&mut self, board: &mut B, msgs: &mut Messages) {
        let mut buf = [0u8; RX_BUFF_SIZE];
        match board.serial_rx_read(&mut buf) {
            Some(Ok(n)) => {
                // defmt::debug!("got {} bytes", n);
                for i in 0..n {
                    if let Some(frame) = self.mav_parser.feed_byte(buf[i]) {
                        // defmt::debug!("got a frame!");
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

    fn send_status(&mut self, board: &mut B, system_id: u8, msg: RosflightStatusMsg) {
        self.send_message(board, system_id, RosflightStatus::from(msg));
    }
    fn send_timesync(&mut self, board: &mut B, system_id: u8, msg: TimesyncMsg) -> bool {
        self.send_message(board, system_id, Timesync::from(msg));
        return true;
    }
    fn send_named_value(&mut self, board: &mut B, system_id: u8, msg: ParamValueMsg) {
        self.send_message(board, system_id, ParamValue::from(msg));
    }
    fn send_heartbeat(&mut self, board: &mut B, system_id: u8, msg: HeartbeatMsg) -> bool {
        self.send_message(board, system_id, messages::Heartbeat::from(msg));
        return true;
    }
    fn send_version(&mut self, board: &mut B, system_id: u8, msg: RosflightVersionMsg) {
        self.send_message(board, system_id, RosflightVersion::from(msg));
    }
    fn send_diff_pressure(&mut self, board: &mut B, system_id: u8, msg: DiffPressureMsg) {
        self.send_message(board, system_id, DiffPressure::from(msg));
    }
    fn send_baro(&mut self, board: &mut B, system_id: u8, msg: SmallBaroMsg) {
        self.send_message(board, system_id, SmallBaro::from(msg));
    }
    fn send_imu(&mut self, board: &mut B, system_id: u8, msg: SmallImuMsg) {
        self.send_message(board, system_id, SmallImu::from(msg));
    }
    fn send_attitude(&mut self, board: &mut B, system_id: u8, msg: AttitudeQuaternionMsg) {
        self.send_message(board, system_id, AttitudeQuaternion::from(msg));
    }
    fn send_output_raw(&mut self, board: &mut B, system_id: u8, msg: RosflightOutputRawMsg) {
        self.send_message(board, system_id, RosflightOutputRaw::from(msg));
    }
    fn send_rc_raw(&mut self, board: &mut B, system_id: u8, msg: RosflightOutputRawMsg) {
        self.send_message(board, system_id, RosflightOutputRaw::from(msg));
    }
    fn send_range(&mut self, board: &mut B, system_id: u8, msg: SmallRangeMsg) {
        self.send_message(board, system_id, SmallRange::from(msg));
    }
    fn send_mag(&mut self, board: &mut B, system_id: u8, msg: SmallMagMsg) {
        self.send_message(board, system_id, SmallMag::from(msg));
    }
    fn send_gnss(&mut self, board: &mut B, system_id: u8, msg: RosflightGnssMsg) {
        self.send_message(board, system_id, RosflightGnss::from(msg));
    }
    fn send_cmd_ack(&mut self, board: &mut B, system_id: u8, msg: RosflightCmdAckMsg) {
        self.send_message(board, system_id, RosflightCmdAck::from(msg));
    }
}



impl From<enums::RosflightAuxCmdType> for comm_enums::RosflightAuxCmdType {
    fn from(val: enums::RosflightAuxCmdType) -> Self {
        use enums::RosflightAuxCmdType as MavCmd;
        use comm_enums::RosflightAuxCmdType as CommCmd;
        match val {
            MavCmd::Disabled => CommCmd::Disabled,
            MavCmd::Motor => CommCmd::Motor,
            MavCmd::Servo => CommCmd::Servo
        }
    }
}

// RECEIVING MSG CONVERSIONS
impl From<messages::RosflightCmd> for RosflightCmdMsg {
    fn from(msg: messages::RosflightCmd) -> Self {
        Self {
            command: comm_enums::RosflightCmd::from(msg.command)
        }
    }
}

impl From<messages::Timesync> for TimesyncMsg {
    fn from(msg: messages::Timesync) -> Self {
        Self {
            tc1: msg.tc1,
            ts1: msg.ts1,
        }
    }
}

impl From<messages::ExternalAttitude> for ExternalAttitudeMsg {
    fn from(msg: messages::ExternalAttitude) -> Self {
        Self {
            qx: msg.qx,
            qw: msg.qw,
            qy: msg.qy,
            qz: msg.qz
        }
    }
}

impl From<messages::OffboardControl> for OffboardControlMsg {
    fn from(msg: messages::OffboardControl) -> Self {
        use enums::OffboardControlMode as MavMode;
        use enums::OffboardControlIgnore as MavIgnore;
        use comm_enums::OffboardControlIgnore as CommIgnore;
        use comm_enums::OffboardControlMode as CommMode;

        // --- Build the new `ignore` flags by checking the bits ---
        let mut comm_ignore = CommIgnore::empty(); // Start with no flags set

        // Cast the incoming enum to a u8 to treat it as a bitmask
        let ignore_mask = msg.ignore as u8;

        // Check each bit position using bitwise AND
        if (ignore_mask & (MavIgnore::IgnoreValue1 as u8)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_QX;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue2 as u8)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_QY;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue3 as u8)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_QZ;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue4 as u8)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_FX;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue5 as u8)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_FY;
        }
        if (ignore_mask & (MavIgnore::IgnoreValue6 as u8)) != 0 {
            comm_ignore |= CommIgnore::IGNORE_FZ;
        }
        // --- End of flag building ---

        Self {
            mode: match msg.mode {
                MavMode::ModePassThrough => CommMode::ModePassThrough,
                MavMode::ModeRollratePitchrateYawrateThrottle => CommMode::ModeRollratePitchrateYawrateThrottle,
                _ => CommMode::ModePassThrough, // Default for other modes
            },
            ignore: comm_ignore, // Use the flags we just built
            qx: msg.qx,
            qy: msg.qy,
            qz: msg.qz,
            fx: msg.fx,
            fy: msg.fy,
            fz: msg.fz,
        }
    }
}

impl From<messages::RosflightAuxCmd> for RosflightAuxCmdMsg {
    fn from(msg: messages::RosflightAuxCmd) -> Self {
        Self {
            type_array: msg.type_array.map(|t| t.into()),
            aux_cmd_array: msg.aux_cmd_array,
        }
    }
}

impl From<messages::Heartbeat> for HeartbeatMsg {
    fn from(msg: messages::Heartbeat) -> Self {
        Self {
            type_: msg.type_,
            autopilot: msg.autopilot,
            base_mode: msg.base_mode,
            custom_mode: msg.custom_mode,
            system_status: msg.system_status,
            mavlink_version: msg.mavlink_version
        }
    }
}

impl From<messages::ParamRequestRead> for ParamRequestReadMsg {
    fn from(msg: messages::ParamRequestRead) -> Self {
        Self {
            target_system: msg.target_system,
            target_component: msg.target_component,
            /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
            param_identifier: match msg.param_index {
                -1 => comm_enums::ParamIdentifier::ID(msg.param_id),
                _ => comm_enums::ParamIdentifier::INDEX(msg.param_index)
            }
        }
    }
}

impl From<messages::ParamRequestList> for ParamRequestListMsg {
    fn from(msg: messages::ParamRequestList) -> Self {
        Self {
            target_system: msg.target_system,
            target_component: msg.target_component
        }
    }
}

impl From<messages::ParamSet> for ParamSetMsg {
    fn from(msg: messages::ParamSet) -> Self {
        use enums::MavParamType::*;
        use crate::params2::ParamValue;
        Self {
            target_system: msg.target_system,
            target_component: msg.target_component,
            param_id: msg.param_id,
            param_value: match msg.param_type {
                Uint8 | Uint16 | Uint32 | Uint64 => ParamValue::Uint(msg.param_value as u32),
                Int8 | Int16 | Int32 | Int64 => ParamValue::Int(msg.param_value as i32),
                Real32 | Real64 => ParamValue::Float(msg.param_value)
            }
        }
    }
}

// SENDING MSG CONVERSIONS

impl From<TimesyncMsg> for messages::Timesync {
    fn from(msg: TimesyncMsg) -> Self {
        Self {
            tc1: msg.tc1,
            ts1: msg.ts1
        }
    }
}

impl From<RosflightStatusMsg> for messages::RosflightStatus {
    fn from(msg: RosflightStatusMsg) -> Self {
        Self {
            armed: msg.armed,
            failsafe: msg.failsafe,
            rc_override: msg.rc_override,
            offboard: msg.offboard,
            error_code: msg.error_code.bits() as u8,
            control_mode: msg.control_mode.into(),
            num_errors: msg.num_errors,
            loop_time_us: msg.loop_time_us
        }
    }
}

impl From<comm_enums::OffboardControlMode> for enums::OffboardControlMode {
    fn from(val: comm_enums::OffboardControlMode) -> Self {
        use comm_enums::OffboardControlMode as CommMode;
        use enums::OffboardControlMode as MavMode;
        match val {
            CommMode::ModePassThrough => MavMode::ModePassThrough,
            CommMode::ModeRollratePitchrateYawrateThrottle => MavMode::ModeRollratePitchrateYawrateThrottle,
        }
    }
}

impl From<RosflightVersionMsg> for messages::RosflightVersion {
    fn from(msg: RosflightVersionMsg) -> Self {
        Self {
            version: msg.version,
        }
    }
}

impl From<SmallImuMsg> for messages::SmallImu {
    fn from(msg: SmallImuMsg) -> Self {
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

impl From<comm_enums::RosflightRangeType> for enums::RosflightRangeType {
    fn from(val: comm_enums::RosflightRangeType) -> Self {
        use comm_enums::RosflightRangeType as CommType;
        use enums::RosflightRangeType as MavType;
        match val {
            CommType::RosflightRangeSonar => MavType::RosflightRangeSonar,
            CommType::RosflightRangeLidar => MavType::RosflightRangeLidar,
        }
    }
}

impl From<SmallRangeMsg> for messages::SmallRange {
    fn from(msg: SmallRangeMsg) -> Self {
        Self {
            type_: msg.type_.into(),
            range: msg.range,
            max_range: msg.max_range,
            min_range: msg.min_range,
        }
    }
}

impl From<SmallMagMsg> for messages::SmallMag {
    fn from(msg: SmallMagMsg) -> Self {
        Self {
            xmag: msg.xmag,
            ymag: msg.ymag,
            zmag: msg.zmag,
        }
    }
}

impl From<SmallBaroMsg> for messages::SmallBaro {
    fn from(msg: SmallBaroMsg) -> Self {
        Self {
            altitude: msg.altitude,
            pressure: msg.pressure,
            temperature: msg.temperature,
        }
    }
}

impl From<HeartbeatMsg> for messages::Heartbeat {
    fn from(msg: HeartbeatMsg) -> Self {
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

impl From<DiffPressureMsg> for messages::DiffPressure {
    fn from(msg: DiffPressureMsg) -> Self {
        Self {
            velocity: msg.velocity,
            diff_pressure: msg.diff_pressure,
            temperature: msg.temperature,
        }
    }
}

impl From<AttitudeQuaternionMsg> for messages::AttitudeQuaternion {
    fn from(msg: AttitudeQuaternionMsg) -> Self {
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

impl From<RosflightOutputRawMsg> for messages::RosflightOutputRaw {
    fn from(msg: RosflightOutputRawMsg) -> Self {
        Self {
            stamp: msg.stamp,
            values: msg.values,
        }
    }
}

impl From<RcChannelsMsg> for messages::RcChannels {
    fn from(msg: RcChannelsMsg) -> Self {
        Self {
            time_boot_ms: msg.time_boot_ms,
            chancount: msg.chancount,
            rssi: msg.rssi,
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
            chan17_raw: msg.channels[16],
            chan18_raw: msg.channels[17],
        }
    }
}

impl From<comm_enums::GnssFixType> for enums::GnssFixType {
    fn from(val: comm_enums::GnssFixType) -> Self {
        use comm_enums::GnssFixType as CommType;
        use enums::GnssFixType as MavType;
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

impl From<RosflightGnssMsg> for messages::RosflightGnss {
    fn from(msg: RosflightGnssMsg) -> Self {
        Self {
            seconds: msg.seconds,
            nanos: msg.nanos,
            fix_type: msg.fix_type.into(),
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

impl From<ParamValueMsg> for messages::ParamValue {
    fn from(msg: ParamValueMsg) -> Self {
        use crate::params2::ParamValue as CommParamValue;
        use enums::MavParamType;
        let (value_f32, value_type) = match msg.param_value {
            // Should the ParamValue type be updated to support smaller types?
            CommParamValue::Float(f) => (f, MavParamType::Real32),
            CommParamValue::Int(i) => (i as f32, MavParamType::Int32),
            CommParamValue::Uint(u) => (u as f32, MavParamType::Uint32),
            CommParamValue::Bool(b) => (if b { 1.0 } else { 0.0 }, MavParamType::Uint32),
            // Not sure if it's okay to pass these as floats if raw byte value is being used
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

impl From<RosflightCmdAckMsg> for messages::RosflightCmdAck {
    fn from(msg: RosflightCmdAckMsg) -> Self {
        Self {
            // Convert the nested enums using their respective From impls
            command: msg.command.into(),
            success: msg.success.into(),
        }
    }
}

impl From<comm_enums::RosflightCmd> for enums::RosflightCmd {
    fn from(val: comm_enums::RosflightCmd) -> Self {
        use enums::RosflightCmd as MavCmd;
        use comm_enums::RosflightCmd as CommCmd;
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

impl From<enums::RosflightCmd> for comm_enums::RosflightCmd {
    fn from(val: enums::RosflightCmd) -> Self {
        use enums::RosflightCmd as MavCmd;
        use comm_enums::RosflightCmd as CommCmd;
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

impl From<comm_enums::RosflightCmdResponse> for enums::RosflightCmdResponse {
    fn from(val: comm_enums::RosflightCmdResponse) -> Self {
        use enums::RosflightCmdResponse as MavCmd;
        use comm_enums::RosflightCmdResponse as CommCmd;
        match val {
            CommCmd::RosflightCmdFailed => MavCmd::RosflightCmdFailed,
            CommCmd::RosflightCmdSuccess => MavCmd::RosflightCmdSuccess,
        }
    }
}



impl From<enums::RosflightCmdResponse> for comm_enums::RosflightCmdResponse {
    fn from(val: enums::RosflightCmdResponse) -> Self {
        use enums::RosflightCmdResponse as MavResponse;
        use comm_enums::RosflightCmdResponse as CommResponse;
        match val {
            MavResponse::RosflightCmdFailed => CommResponse::RosflightCmdFailed,
            MavResponse::RosflightCmdSuccess => CommResponse::RosflightCmdSuccess
        }
    }
}