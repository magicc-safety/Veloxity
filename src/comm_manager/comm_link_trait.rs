// /**
// ******************************************************************************
// * File     : comm_link_trait.rs
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
#[cfg(feature = "nucleo")]
pub mod basic_processor;
#[cfg(feature = "nucleo")]
pub mod mavlink;

use crate::board;
use crate::packets;
use crate::params;

pub trait CommInterface<B: board::Board> {
    fn send_heartbeat(&mut self, board: &B, system_id: u8, fixed_wing: bool) -> bool;
    fn send_named_value(
        &mut self,
        board: &B,
        system_id: u8,
        timestamp_ms: u32,
        name: &[u8],
        value: params::ParamValue,
    );
    fn send_status(
        &mut self,
        board: &B,
        system_id: u8,
        armed: bool,
        failsafe: bool,
        rc_override: bool,
        offboard: bool,
        error_code: u8,
        control_mode: u8,
        num_errors: i16,
        loop_time_us: i16,
    );
    fn send_timesync(&mut self, board: &B, system_id: u8, tc1: i64, ts1: i64) -> bool;
    fn send_version(&mut self, board: &B, system_id: u8, version: &[u8]);

    fn send_output_raw(
        &mut self,
        board: &B,
        system_id: u8,
        timestamp_ms: u32,
        raw_outputs: [f32; 14],
    );
    fn send_attitude(&mut self, board: &B, system_id: u8, packet: &packets::AttitudePacket);
    fn send_baro(&mut self, board: &B, sysem_id: u8, packet: &packets::BaroPacket);
    fn send_diff_pressure(&mut self, board: &B, system_id: u8, packet: &packets::PitotPacket);
    fn send_imu(&mut self, board: &B, system_id: u8, packet: &packets::ImuPacket);
    fn send_log_message(&mut self, board: &B, system_id: u8, packet: &packets::LogPacket);
    fn send_mag(&mut self, board: &B, system_id: u8, packet: &packets::MagPacket);
    fn send_rc_raw(&mut self, board: &B, system_id: u8, packet: &packets::RcPacket);
    fn send_range(&mut self, board: &B, system_id: u8, packet: &packets::RangePacket);
    fn send_gnss(&mut self, board: &B, system_id: u8, data: &packets::GNSSPacket);
    fn send_gnss_full(&mut self, board: &B, system_id: u8, data: &packets::GNSSPacket);
    fn handle_incoming_messages(&mut self);
}

#[allow(async_fn_in_trait)]
pub trait EmbeddedComInterface {
    async fn process_bytes(&mut self, buf: &[u8], num_bytes: usize);
}
