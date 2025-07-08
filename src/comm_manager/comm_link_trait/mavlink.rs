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
use crate::board;
use crate::comm_manager::comm_link_trait::CommInterface;
use crate::comm_manager::mavlink_parser;
use crate::mavlink::dialects::rosflight::{messages, Rosflight};
use mavio::prelude::*;
use mavio::Frame;

static RX_BUFF_SIZE: usize = 2048;

// only include options for messages you'd receive...
pub struct MavlinkInterface {
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
        //defmt::trace!(
        //    "🎉 Heartbeat: autopilot={}, mode={}, status={}, custom_mode: {}",
        //    msg.autopilot,
        //    msg.base_mode,
        //    msg.system_status,
        //    msg.custom_mode,
        //);
        //self.heartbeat = Some(msg);
    }
}

impl<B: board::Board> CommInterface<B> for MavlinkInterface {
    fn handle_incoming_messages(&mut self, board: &mut B) {
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
    }
    fn send_timesync(&mut self, board: &mut B, system_id: u8, tc1: i64, ts1: i64) -> bool {
        let mut buf = [0u8; 100];
        //let byte_count = telem::heartbeat(&mut buf);

        let timesync = messages::Timesync { tc1, ts1 };

        let frame = match Frame::builder()
            .version(V1)
            .sequence(0)
            .system_id(1)
            .component_id(1)
            .message(&timesync)
        {
            Ok(builder) => builder.build(),
            Err(_) => {
                //defmt::trace!("Heartbeat: Error with FrameBuilder!");
                return false;
            }
        };

        if frame.body_length() > buf.len() {
            //defmt::trace!("Heartbeat: Body Length Error");
            return false;
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

        board.serial_tx_write(&buf[..pos]);
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
    }
    fn send_heartbeat(&mut self, board: &mut B, system_id: u8, fixed_wing: bool) -> bool {
        let mut buf = [0u8; 100];
        //let byte_count = telem::heartbeat(&mut buf);

        let heartbeat = messages::Heartbeat {
            autopilot: 1,
            base_mode: 1,
            type_: 1,
            custom_mode: 1,
            system_status: 1,
            mavlink_version: 1,
        };

        let frame = match Frame::builder()
            .version(V1)
            .sequence(0)
            .system_id(1)
            .component_id(1)
            .message(&heartbeat)
        {
            Ok(builder) => builder.build(),
            Err(_) => {
                //defmt::trace!("Heartbeat: Error with FrameBuilder!");
                return false;
            }
        };

        if frame.body_length() > buf.len() {
            //defmt::trace!("Heartbeat: Body Length Error");
            return false;
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
        return true;
    }
    fn send_version(&mut self, board: &mut B, system_id: u8, version: &[u8]) {}
    fn send_diff_pressure(
        &mut self,
        board: &mut B,
        system_id: u8,
        packet: &crate::packets::PitotPacket,
    ) {
    }
    fn send_baro(&mut self, board: &mut B, sysem_id: u8, packet: &crate::packets::BaroPacket) {}
    fn send_imu(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::ImuPacket) {}
    fn send_attitude(
        &mut self,
        board: &mut B,
        system_id: u8,
        packet: &crate::packets::AttitudePacket,
    ) {
    }
    fn send_log_message(
        &mut self,
        board: &mut B,
        system_id: u8,
        packet: &crate::packets::LogPacket,
    ) {
    }
    fn send_output_raw(
        &mut self,
        board: &mut B,
        system_id: u8,
        timestamp_ms: u32,
        raw_outputs: [f32; 14],
    ) {
    }
    fn send_rc_raw(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::RcPacket) {}
    fn send_range(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::RangePacket) {}
    fn send_mag(&mut self, board: &mut B, system_id: u8, packet: &crate::packets::MagPacket) {}
    fn send_gnss(&mut self, board: &mut B, system_id: u8, data: &crate::packets::GNSSPacket) {}
    fn send_gnss_full(&mut self, board: &mut B, system_id: u8, data: &crate::packets::GNSSPacket) {}
}
