// ******************************************************************************
// * File     : comms/veloxity_mavlink/src/conversions.rs
// * Date     : June 28, 2026
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

use crate::generated::dialects::rosflight::{enums as mav_enums, messages as mav_messages};
use veloxity_core::comm::messages::{enums as comm_enums, messages as core_messages};
use veloxity_core::packets;

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
impl From<mav_messages::RosflightCmd> for core_messages::RosflightCmdMsg {
    fn from(msg: mav_messages::RosflightCmd) -> Self {
        Self {
            command: comm_enums::RosflightCmd::from(msg.command),
        }
    }
}

impl From<mav_messages::Timesync> for core_messages::TimesyncMsg {
    fn from(msg: mav_messages::Timesync) -> Self {
        Self {
            tc1: msg.tc1,
            ts1: msg.ts1,
        }
    }
}

impl From<mav_messages::ExternalAttitude> for core_messages::ExternalAttitudeMsg {
    fn from(msg: mav_messages::ExternalAttitude) -> Self {
        Self {
            qx: msg.qx,
            qw: msg.qw,
            qy: msg.qy,
            qz: msg.qz,
        }
    }
}

impl From<mav_messages::OffboardControl> for core_messages::OffboardControlMsg {
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

impl From<mav_messages::RosflightAuxCmd> for core_messages::RosflightAuxCmdMsg {
    fn from(msg: mav_messages::RosflightAuxCmd) -> Self {
        Self {
            type_array: msg.type_array.map(comm_enums::RosflightAuxCmdType::from),
            aux_cmd_array: msg.aux_cmd_array,
        }
    }
}

impl From<mav_messages::Heartbeat> for core_messages::HeartbeatMsg {
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

impl From<mav_messages::ParamRequestRead> for core_messages::ParamRequestReadMsg {
    fn from(msg: mav_messages::ParamRequestRead) -> Self {
        Self {
            target_system: msg.target_system,
            target_component: msg.target_component,
            param_identifier: match msg.param_index {
                -1 => comm_enums::ParamIdentifier::ID(msg.param_id),
                _ => comm_enums::ParamIdentifier::INDEX(msg.param_index),
            },
        }
    }
}

impl From<mav_messages::ParamRequestList> for core_messages::ParamRequestListMsg {
    fn from(msg: mav_messages::ParamRequestList) -> Self {
        Self {
            target_system: msg.target_system,
            target_component: msg.target_component,
        }
    }
}

impl From<mav_messages::ParamSet> for core_messages::ParamSetMsg {
    fn from(msg: mav_messages::ParamSet) -> Self {
        use mav_enums::MavParamType::*;
        use veloxity_core::params::ParamValue;
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

impl From<core_messages::TimesyncMsg> for mav_messages::Timesync {
    fn from(msg: core_messages::TimesyncMsg) -> Self {
        mav_messages::Timesync {
            tc1: msg.tc1,
            ts1: msg.ts1,
        }
    }
}

impl From<core_messages::RosflightStatusMsg> for mav_messages::RosflightStatus {
    fn from(msg: core_messages::RosflightStatusMsg) -> Self {
        mav_messages::RosflightStatus {
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

impl From<core_messages::RosflightVersionMsg> for mav_messages::RosflightVersion {
    fn from(msg: core_messages::RosflightVersionMsg) -> Self {
        mav_messages::RosflightVersion {
            version: msg.version,
        }
    }
}

impl From<core_messages::SmallImuMsg> for mav_messages::SmallImu {
    fn from(msg: core_messages::SmallImuMsg) -> Self {
        mav_messages::SmallImu {
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

impl From<core_messages::SmallRangeMsg> for mav_messages::SmallRange {
    fn from(msg: core_messages::SmallRangeMsg) -> Self {
        mav_messages::SmallRange {
            type_: msg.type_.into(),
            range: msg.range,
            max_range: msg.max_range,
            min_range: msg.min_range,
        }
    }
}

impl From<core_messages::SmallMagMsg> for mav_messages::SmallMag {
    fn from(msg: core_messages::SmallMagMsg) -> Self {
        mav_messages::SmallMag {
            xmag: msg.xmag,
            ymag: msg.ymag,
            zmag: msg.zmag,
        }
    }
}

impl From<core_messages::SmallBaroMsg> for mav_messages::SmallBaro {
    fn from(msg: core_messages::SmallBaroMsg) -> Self {
        mav_messages::SmallBaro {
            altitude: msg.altitude,
            pressure: msg.pressure,
            temperature: msg.temperature,
        }
    }
}

impl From<core_messages::HeartbeatMsg> for mav_messages::Heartbeat {
    fn from(msg: core_messages::HeartbeatMsg) -> Self {
        mav_messages::Heartbeat {
            type_: msg.type_,
            autopilot: msg.autopilot,
            base_mode: msg.base_mode,
            custom_mode: msg.custom_mode,
            system_status: msg.system_status,
            mavlink_version: msg.mavlink_version,
        }
    }
}

impl From<core_messages::DiffPressureMsg> for mav_messages::DiffPressure {
    fn from(msg: core_messages::DiffPressureMsg) -> Self {
        mav_messages::DiffPressure {
            velocity: msg.velocity,
            diff_pressure: msg.diff_pressure,
            temperature: msg.temperature,
        }
    }
}

impl From<core_messages::AttitudeQuaternionMsg> for mav_messages::AttitudeQuaternion {
    fn from(msg: core_messages::AttitudeQuaternionMsg) -> Self {
        mav_messages::AttitudeQuaternion {
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

impl From<core_messages::RosflightOutputRawMsg> for mav_messages::RosflightOutputRaw {
    fn from(msg: core_messages::RosflightOutputRawMsg) -> Self {
        mav_messages::RosflightOutputRaw {
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

impl From<core_messages::RosflightGnssMsg> for mav_messages::RosflightGnss {
    fn from(msg: core_messages::RosflightGnssMsg) -> Self {
        mav_messages::RosflightGnss {
            seconds: msg.seconds,
            nanos: msg.nanos,
            fix_type: msg.fix_type.into(),
            num_sat: msg.num_sat,
            lat: msg.lat,
            lon: msg.lon,
            // ROSFLIGHT_GNSS names this wire field `height`, but ROSflight C and
            // rosflight_msgs define its value as altitude above mean sea level.
            height: msg.height_msl,
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

impl From<core_messages::ParamValueMsg> for mav_messages::ParamValue {
    fn from(msg: core_messages::ParamValueMsg) -> Self {
        use mav_enums::MavParamType;
        use veloxity_core::params::ParamValue as CommParamValue;
        let (value_f32, value_type) = match msg.param_value {
            // Should the ParamValue type be updated to support smaller types?
            CommParamValue::Float(f) => (f, MavParamType::Real32),
            CommParamValue::Int(i) => (f32::from_bits(i as u32), MavParamType::Int32),
            CommParamValue::Uint(u) => (f32::from_bits(u), MavParamType::Uint32),
            CommParamValue::Bool(b) => {
                (f32::from_bits(if b { 1 } else { 0 }), MavParamType::Uint32)
            } // Not sure if it's okay to pass these as floats if raw byte value is being used
        };
        mav_messages::ParamValue {
            param_id: msg.param_id,
            param_value: value_f32,
            param_type: value_type,
            param_count: msg.param_count,
            param_index: msg.param_index,
        }
    }
}

impl From<core_messages::RosflightCmdAckMsg> for mav_messages::RosflightCmdAck {
    fn from(msg: core_messages::RosflightCmdAckMsg) -> Self {
        mav_messages::RosflightCmdAck {
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

impl From<core_messages::RcChannelsMsg> for mav_messages::RcChannels {
    fn from(msg: core_messages::RcChannelsMsg) -> Self {
        // `msg.channels` is [u16; 16] (from RC_PACKET_CHANNELS)
        // `Self` (RcChannels) has 18 individual channel fields.

        mav_messages::RcChannels {
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

impl From<core_messages::BatteryStatusMsg> for mav_messages::RosflightBatteryStatus {
    fn from(msg: core_messages::BatteryStatusMsg) -> Self {
        mav_messages::RosflightBatteryStatus {
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

impl From<core_messages::StatustextMsg> for mav_messages::Statustext {
    fn from(msg: core_messages::StatustextMsg) -> Self {
        mav_messages::Statustext {
            severity: msg.severity.into(),
            text: msg.text,
        }
    }
}

impl From<core_messages::RosflightHardErrorMsg> for mav_messages::RosflightHardError {
    fn from(msg: core_messages::RosflightHardErrorMsg) -> Self {
        mav_messages::RosflightHardError {
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
    fn gnss_msl_altitude_uses_existing_mavlink_height_field() {
        let converted = mav_messages::RosflightGnss::from(core_messages::RosflightGnssMsg {
            seconds: 1_700_000_000,
            nanos: 123,
            fix_type: packets::GNSSFixType::ThreeD,
            num_sat: 12,
            lat: 40.0,
            lon: -111.0,
            height_msl: 1_402.25,
            vel_n: 0.0,
            vel_e: 0.0,
            vel_d: 0.0,
            h_acc: 0.9,
            v_acc: 1.2,
            s_acc: 0.1,
            rosflight_timestamp: 42,
        });

        assert_eq!(converted.height, 1_402.25);
    }

    #[test]
    fn offboard_control_conversion_preserves_roll_pitch_mode() {
        let msg = mav_messages::OffboardControl {
            mode: mav_enums::OffboardControlMode::ModeRollPitchYawrateThrottle,
            ignore: mav_enums::OffboardControlIgnore::IgnoreNone,
            u: [0.4, 0.5, 0.6, 0.1, 0.2, 0.3, 0.7, 0.8, 0.9, 1.0],
        };

        let converted = core_messages::OffboardControlMsg::from(msg);

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

            let converted = core_messages::OffboardControlMsg::from(msg);

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

        let converted = core_messages::OffboardControlMsg::from(msg);

        assert_eq!(converted.passthrough, [0.6, 0.7, 0.8, 0.9]);
        assert_eq!(
            converted.ignore,
            comm_enums::OffboardControlIgnore::IGNORE_PASS_3
        );
    }
}
