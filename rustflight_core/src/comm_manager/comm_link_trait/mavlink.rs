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
//use defmt;

use crate::mavlink::dialects::rosflight::messages::SmallMag;
use crate::{board, packets};
use crate::comm_manager::comm_link_trait::CommInterface;
use crate::comm_manager::mavlink_parser;
use crate::comm_messages;
use crate::mavlink::dialects::rosflight::{Rosflight, messages, enums};
use mavio::Frame;
use mavio::prelude::*;

static RX_BUFF_SIZE: usize = 2048;

// only include options for messages you'd receive...

pub struct MavlinkInterface {
    pub system_id: u8,
    pub component_id: u8,
    sequence: u8,
    pub param_set: Option<messages::ParamSet>,
    pub external_attitude: Option<messages::ExternalAttitude>,
    pub time_sync: Option<messages::Timesync>,
    pub param_request_read: Option<messages::ParamRequestRead>,
    pub rosflight_cmd: Option<messages::RosflightCmd>,
    pub param_request_list: Option<messages::ParamRequestList>,
    pub heartbeat: Option<messages::Heartbeat>,
    pub rosflight_aux_cmd: Option<messages::RosflightAuxCmd>,
    pub offboard_control: Option<messages::OffboardControl>,
    mav_parser: mavlink_parser::MavlinkParser,
}

impl MavlinkInterface {
    pub fn new() -> Self {
        Self {
            system_id: 1, // Should this be passed into each function or hardcoded?
            component_id: 1,
            sequence: 0,
            param_set: None,
            external_attitude: None,
            time_sync: None,
            param_request_read: None,
            rosflight_cmd: None,
            param_request_list: None,
            heartbeat: None,
            rosflight_aux_cmd: None,
            offboard_control: None,
            mav_parser: mavlink_parser::MavlinkParser::new(),
        }
    }

    pub fn handle_msg_param_request_list(&mut self, msg: messages::ParamRequestList) {
        //defmt::trace!(
        //    "Parameter (list) with ID: {} requested!",
        //    msg.target_component
        //);
        self.param_request_list = Some(msg);
    }

    pub fn handle_msg_param_request_read(&mut self, msg: messages::ParamRequestRead) {
        if msg.param_index == -1 {
            let end = msg
                .param_id
                .iter()
                .position(|&b| b == 0)
                .unwrap_or(msg.param_id.len());
            let valid_bytes = &msg.param_id[..end];
            match core::str::from_utf8(valid_bytes) {
                Ok(param_id) => {
                    //defmt::trace!("Parameter (read) name {} requested", param_id);
                }
                Err(e) => {
                    //defmt::trace!("Parameter (send) invalind name");
                }
            }
        } else {
            //defmt::trace!("Parameter (read) index {} requested!", msg.param_index);
        }

        self.param_request_read = Some(msg);
    }

    pub fn handle_msg_param_set(&mut self, msg: messages::ParamSet) {
        self.param_set = Some(msg)
    }

    pub fn handle_msg_offboard_control(&mut self, msg: messages::OffboardControl) {
        self.offboard_control = Some(msg);
    }

    pub fn handle_msg_external_attitude(&mut self, msg: messages::ExternalAttitude) {
        self.external_attitude = Some(msg);
    }

    pub fn handle_msg_rosflight_cmd(&mut self, msg: messages::RosflightCmd) {
        self.rosflight_cmd = Some(msg);
    }

    pub fn handle_msg_rosflight_aux_cmd(&mut self, msg: messages::RosflightAuxCmd) {
        self.rosflight_aux_cmd = Some(msg);
    }

    pub fn handle_msg_timesync(&mut self, msg: messages::Timesync) {
        //defmt::trace!("Timesync: {}", msg.ts1);
        self.time_sync = Some(msg);
    }

    pub fn handle_msg_heartbeat(&mut self, msg: messages::Heartbeat) {
        //defmt::debug!(
        //    "🎉 Heartbeat: autopilot={}, mode={}, status={}, custom_mode: {}",
        //    msg.autopilot,
        //    msg.base_mode,
        //    msg.system_status,
        //    msg.custom_mode,
        //);
        self.heartbeat = Some(msg);
    }

    fn frame_builder<T: Message>(&mut self, msg: &T) -> mavio::Result<Frame<V1>> {
        let frame = Frame::builder()
            .version(V1)
            .system_id(self.system_id)
            .component_id(self.component_id)
            .sequence(self.sequence)
            .message(msg)?
            .build();

        // Increment the sequence number, wrapping on overflow
        self.sequence = self.sequence.wrapping_add(1);

        Ok(frame)
    }

    fn send_message<B: board::BoardTrait, T: Message>(&mut self, board: &mut B, msg: &T) {
        let frame = match self.frame_builder(msg) {
            Ok(f) => f,
            Err(_) => {
                //defmt::trace!("Error with FrameBuilder!");
                return;
            }
        };

        // MAVLink V1 specification shows up to 263 bytes possible (Previously 100). Is this right?
        let mut buf = [0u8; 263]; 

        if frame.body_length() > buf.len() {
            //defmt::trace!("Body Length Error");
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
}

impl<B: board::BoardTrait> CommInterface<B> for MavlinkInterface {
    fn handle_incoming_messages(&mut self, board: &mut B, messages: &mut comm_messages::Messages) {
        let mut buf = [0u8; RX_BUFF_SIZE];
        match board.serial_rx_read(&mut buf) {
            Some(Ok(n)) => {
                //defmt::trace!("Heartbeat: got {} bytes", n);
                for i in 0..n {
                    if let Some(frame) = self.mav_parser.feed_byte(buf[i]) {
                        //defmt::trace!("Heartbeat: got a frame!");
                        if let Some(message) = mavlink_parser::process_mavlink_frame(frame) {
                            match (message) {
                                Rosflight::ParamSet(ps) => {
                                    self.handle_msg_param_set(ps);
                                }
                                Rosflight::ExternalAttitude(es) => {
                                    self.handle_msg_external_attitude(es);
                                }
                                Rosflight::Timesync(ts) => {
                                    self.handle_msg_timesync(ts);
                                }
                                Rosflight::ParamRequestRead(pr) => {
                                    self.handle_msg_param_request_read(pr);
                                }
                                Rosflight::RosflightCmd(cmd) => {
                                    self.handle_msg_rosflight_cmd(cmd);
                                }
                                Rosflight::ParamRequestList(pl) => {
                                    self.handle_msg_param_request_list(pl);
                                }
                                Rosflight::Heartbeat(hb) => {
                                    self.handle_msg_heartbeat(hb);
                                }
                                Rosflight::RosflightAuxCmd(cmd) => {
                                    self.handle_msg_rosflight_aux_cmd(cmd);
                                }
                                Rosflight::OffboardControl(oc) => {
                                    self.handle_msg_offboard_control(oc);
                                }
                                _ => {
                                    //defmt::trace!("System: Other ROSflight message received");
                                }
                            }
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
        armed: bool,
        failsafe: bool,
        rc_override: bool,
        offboard: bool,
        error_code: u8,
        control_mode: u8,
        num_errors: i16,
        loop_time_us: i16,
    ) {
        // TODO: Figure out how to convert to Rosflight error types
        // let status = messages::RosflightStatus {
        //     armed: if armed {1} else {0},
        //     failsafe: if failsafe {1} else {0}, 
        //     rc_override: if rc_override {1} else {0},
        //     offboard: if offboard {1} else {0},
        //     error_code: error_code, // convert to RosflightErrorCode?
        //     control_mode: control_mode, // convert to OffboardControlMode?
        //     num_errors: num_errors,
        //     loop_time_us: loop_time_us,
        // };
        // self.send_message(board, &status);
    }
    fn send_timesync(&mut self, board: &mut B, system_id: u8, tc1: i64, ts1: i64) -> bool {
        let timesync = messages::Timesync { tc1, ts1 };
        self.send_message(board, &timesync);
        return true;
    }
    fn send_named_value(
        &mut self,
        board: &mut B,
        system_id: u8,
        timestamp_ms: u32,
        name: &[u8],
        value: crate::params::ParamValue,
    ) {
        // TODO: Parameters
    }
    fn send_heartbeat(&mut self, board: &mut B, system_id: u8, fixed_wing: bool) -> bool {
        let heartbeat = messages::Heartbeat {
            autopilot: 1,
            base_mode: 1,
            type_: 1,
            custom_mode: 1,
            system_status: 1,
            mavlink_version: 1,
        };
        self.send_message(board, &heartbeat);
        return true;
    }
    fn send_version(&mut self, board: &mut B, system_id: u8, version: &[u8]) {
        let version = messages::RosflightVersion {
            version: [0; 50] // TODO: Figure out what to put here
        };
        self.send_message(board, &version);
    }
    fn send_diff_pressure(
        &mut self,
        board: &mut B,
        system_id: u8,
        packet: &crate::packets::PitotPacket,
    ) {
        // TODO: velocity?
        // let diff = messages::DiffPressure {
        //     velocity: idk,
        //     diff_pressure: packet.pressure,
        //     temperature: packet.temperature
        // };
        // self.send_message(board, &diff);
    }
    fn send_baro(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::BaroPacket) {
        // TODO: altitude?
        // let baro = messages::SmallBaro {
        //     altitude: idk, 
        //     pressure: packet.pressure,
        //     temperature: packet.temperature,
        // };
        // self.send_message(board, &baro);
    }
    fn send_imu(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::ImuPacket) {
        // TODO: make sure acc and gyro ordering is correct. Figure out where time_boot comes from.
        // let imu = messages::SmallImu {
        //     time_boot_us: idk,
        //     xacc: packet.accel[0],
        //     yacc: packet.accel[1],
        //     zacc: packet.accel[2],
        //     xgyro: packet.gyro[0],
        //     ygyro: packet.gyro[1],
        //     zgyro: packet.gyro[2],
        //     temperature: packet.temperature
        // };
        // self.send_message(board, &imu);
    }
    fn send_attitude(
        &mut self,
        board: &mut B,
        system_id: u8,
        packet: &crate::packets::AttitudePacket,
    ) {
        // TODO: Time boot, check ordering
        // let attitude = messages::AttitudeQuaternion {
        //     time_boot_ms: idk,
        //     q1: packet.q[0],
        //     q2: packet.q[1],
        //     q3: packet.q[2],
        //     q4: packet.q[3],
        //     rollspeed: packet.rate[0],
        //     pitchspeed: packet.rate[1],
        //     yawspeed: packet.rate[2]
        // };
        // self.send_message(board, &attitude);
    }
    //fn send_log_message(
    //    &mut self,
    //    board: &mut B,
    //    system_id: u8,
    //    packet: &crate::packets::LogPacket,
    //) {
    //}
    fn send_output_raw(
        &mut self,
        board: &mut B,
        system_id: u8,
        timestamp_ms: u32,
        raw_outputs: [f32; 14],
    ) {
        // TODO: u64 to u32 timestamp
        // let output = messages::RosflightOutputRaw {
        //     stamp: timestamp_ms,
        //     values: raw_outputs
        // };
        // self.send_message(board, &output);
    }
    fn send_rc_raw(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::RcPacket) {
        // TODO: ROSflight packet is hardcoded to 18 channels. RC packet is not.
    }
    fn send_range(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::RangePacket) {
        let rosflight_range: enums::RosflightRangeType = match packet.range_type {
            packets::RangeType::Sonar => enums::RosflightRangeType::RosflightRangeSonar,
            packets::RangeType::Lidar => enums::RosflightRangeType::RosflightRangeLidar,
        };
        let range = messages::SmallRange {
            type_: rosflight_range,
            range: packet.range,
            max_range: packet.max_range,
            min_range: packet.min_range
        };
        self.send_message(board, &range);
    }
    fn send_mag(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::MagPacket) {
        // TODO: Check order
        let mag = messages::SmallMag {
            xmag: packet.flux[0],
            ymag: packet.flux[1],
            zmag: packet.flux[2]
        };
        self.send_message(board, &mag);
    }
    fn send_gnss(&mut self, board: &mut B, system_id: u8, data: &crate::packets::GNSSPacket) {
        // let gnss = messages::RosflightGnss {
        //     // TODO: Some type conversions needed
        //     seconds: data.sec,
        //     nanos: data.nano,
        //     fix_type: data.fix_type,
        //     num_sat: data.num_sats,
        //     lat: data.lat,
        //     lon: data.lon,
        //     height: data.height,
        //     vel_n: data.vel_n,
        //     vel_e: data.vel_e,
        //     vel_d: data.vel_d,
        //     h_acc: data.h_acc,
        //     v_acc: data.v_acc,
        //     s_acc: data.s_acc,
        //     rosflight_timestamp: data.header.timestamp // Is this right?
        // };
        // self.send_message(board, &gnss);
    }
    fn send_gnss_full(&mut self, board: &mut B, system_id: u8, data: &crate::packets::GNSSPacket) {
        // TODO: What is the difference between full and not full?
    }
}