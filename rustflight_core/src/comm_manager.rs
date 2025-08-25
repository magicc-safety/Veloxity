// /**
// ******************************************************************************
// * File     : comm_manager.rs
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
// * list of conditions and the following disclaimer. *
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
pub mod comm_link_trait;
pub mod mavlink_parser;

use crate::board;
use crate::comm_messages::{self, messages::*, enums::*};
use crate::params::ParamValue;
use core::marker::PhantomData;

pub struct CommManager<B, T>
where
    B: board::BoardTrait,
    T: comm_link_trait::CommInterface<B>,
{
    sysid: u8,
    comm_link: T,
    pub msgs: comm_messages::Messages,
    _board_marker: PhantomData<B>,
}

impl<B, T> CommManager<B, T>
where
    B: board::BoardTrait,
    T: comm_link_trait::CommInterface<B>,
{
    pub fn new(comm_link: T) -> Self {
        CommManager {
            sysid: 0,
            comm_link,
            msgs: comm_messages::Messages::default(),
            _board_marker: PhantomData,
        }
    }

    pub fn process_incoming_messages(&mut self, board: &mut B) {
        self.comm_link
            .handle_incoming_messages(board, &mut self.msgs);
    }

    // SENDING PLACEHOLDER MESSAGES
    pub fn send_heartbeat(&mut self, board: &mut B) {
        let msg = HeartbeatMsg { type_: 1, autopilot: 1, base_mode: 1, custom_mode: 1, system_status: 1, mavlink_version: 1 };
        self.comm_link.send_heartbeat(board, self.sysid, msg);
    }

    pub fn send_timesync(&mut self, board: &mut B) {
        let msg = TimesyncMsg { tc1: 1, ts1: 1 };
        self.comm_link.send_timesync(board, self.sysid, msg);
    }

    pub fn send_status(&mut self, board: &mut B) {
        let msg = RosflightStatusMsg {
            armed: 0,
            failsafe: 0,
            rc_override: 0,
            offboard: 0,
            error_code: RosflightErrorCode::RosflightErrorNone,
            control_mode: OffboardControlMode::ModePassThrough,
            num_errors: 0,
            loop_time_us: 0
        };
        self.comm_link.send_status(board, self.sysid, msg);
    }

    pub fn send_named_value(&mut self, board: &mut B) {
        let msg = ParamValueMsg {
            param_id: *b"TEST_PARAM_ID___",
            param_value: ParamValue::Float(123.45),
            param_count: 1,
            param_index: 0,
        };
        self.comm_link.send_named_value(board, self.sysid, msg);
    }

    pub fn send_version(&mut self, board: &mut B) {
        let msg = RosflightVersionMsg {
            version: [42; 50], // Fill with arbitrary byte value
        };
        self.comm_link.send_version(board, self.sysid, msg);
    }

    pub fn send_output_raw(&mut self, board: &mut B) {
        let msg = RosflightOutputRawMsg {
            stamp: 123456789,
            values: [0.5; 14],
        };
        self.comm_link.send_output_raw(board, self.sysid, msg);
    }

    pub fn send_attitude(&mut self, board: &mut B) {
        let msg = AttitudeQuaternionMsg {
            time_boot_ms: 1000,
            q1: 1.0, // w
            q2: 0.0, // x
            q3: 0.0, // y
            q4: 0.0, // z
            rollspeed: 0.1,
            pitchspeed: 0.2,
            yawspeed: 0.3,
        };
        self.comm_link.send_attitude(board, self.sysid, msg);
    }

    pub fn send_baro(&mut self, board: &mut B) {
        let msg = SmallBaroMsg {
            altitude: 123.4,
            pressure: 101.325,
            temperature: 25.0,
        };
        self.comm_link.send_baro(board, self.sysid, msg);
    }

    pub fn send_diff_pressure(&mut self, board: &mut B) {
        let msg = DiffPressureMsg {
            velocity: 15.5,
            diff_pressure: 50.2,
            temperature: 26.0,
        };
        self.comm_link.send_diff_pressure(board, self.sysid, msg);
    }

    pub fn send_imu(&mut self, board: &mut B) {
        let msg = SmallImuMsg {
            time_boot_us: 50000,
            xacc: 0.01,
            yacc: 0.02,
            zacc: 9.8,
            xgyro: 0.001,
            ygyro: 0.002,
            zgyro: 0.003,
            temperature: 45.5,
        };
        self.comm_link.send_imu(board, self.sysid, msg);
    }

    pub fn send_mag(&mut self, board: &mut B) {
        let msg = SmallMagMsg {
            xmag: 0.1,
            ymag: 0.2,
            zmag: 0.3,
        };
        self.comm_link.send_mag(board, self.sysid, msg);
    }

    pub fn send_rc_raw(&mut self, board: &mut B) {
        let msg = RosflightOutputRawMsg {
            stamp: 987654321,
            values: [0.7; 14],
        };
        self.comm_link.send_rc_raw(board, self.sysid, msg);
    }

    pub fn send_range(&mut self, board: &mut B) {
        let msg = SmallRangeMsg {
            type_: RosflightRangeType::RosflightRangeLidar,
            range: 5.5,
            max_range: 40.0,
            min_range: 0.1,
        };
        self.comm_link.send_range(board, self.sysid, msg);
    }

    pub fn send_gnss(&mut self, board: &mut B) {
        let msg = RosflightGnssMsg {
            seconds: 12345,
            nanos: 0,
            fix_type: GnssFixType::GnssFix3dFix,
            num_sat: 12,
            lat: 40.2338,
            lon: -111.6585,
            height: 1400.0,
            vel_n: 0.1,
            vel_e: -0.1,
            vel_d: 0.0,
            h_acc: 1.5,
            v_acc: 2.5,
            s_acc: 0.5,
            rosflight_timestamp: 1,
        };
        self.comm_link.send_gnss(board, self.sysid, msg);
    }
}
