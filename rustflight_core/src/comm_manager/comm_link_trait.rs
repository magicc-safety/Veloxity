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
pub mod mavlink;

use crate::board;
use crate::comm_messages::{self, messages::*};
use crate::packets;
use crate::params2;

pub trait CommInterface<B: board::BoardTrait> {
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
    fn send_rc_raw(&mut self, board: &mut B, system_id: u8, msg: RosflightOutputRawMsg);
    fn send_range(&mut self, board: &mut B, system_id: u8, msg: SmallRangeMsg);
    fn send_gnss(&mut self, board: &mut B, system_id: u8, msg: RosflightGnssMsg);
    fn send_cmd_ack(&mut self, board: &mut B, system_id: u8, msg: RosflightCmdAckMsg);

    fn handle_incoming_messages(&mut self, board: &mut B, msgs: &mut comm_messages::Messages);
}

#[allow(async_fn_in_trait)]
pub trait EmbeddedComInterface {
    async fn process_bytes(&mut self, buf: &[u8], num_bytes: usize);
}
