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
use crate::comm_messages::{self, enums::*, messages::*};
use crate::command_manager::CommandManager;
use crate::events::{
    AuxCommandReceived, BoardCommandRequested, CalibrationRequested, CommEventQueues, CommResponse,
    CommandEventQueues, CompanionEventQueues, CompanionHeartbeatReceived, ConfigInfoRequested,
    ExternalAttitudeReceived, OffboardControlRequested, ParamDefaultsRequested, ParamEventQueues,
    ParamListRequested, ParamReadRequested, ParamSetRequested, RcTrimCalibrationRequested,
    ResetOriginRequested, VersionRequested,
};
use crate::estimator::AttitudeStateTrait;
use crate::mavlink::dialects::Rosflight;
use crate::packets::RC_PACKET_CHANNELS;
use crate::params2::{ParamId, ParamValue, Params};
use crate::sensorprocessors::CalibrationFlags;
use crate::sensors::ProcessedSensors;
use crate::state_machine::StateManager;
use core::marker::PhantomData;

const HEARTBEAT_INTERVAL_US: u64 = 1_000_000; // 1 second = 1,000,000 microseconds
const STATUS_INTERVAL_US: u64 = 500_000; // 2 Hz
const ATTITUDE_INTERVAL_US: u64 = 10_000; // 100 Hz
const IMU_INTERVAL_US: u64 = 2500; // 400 Hz
const BARO_INTERVAL_US: u64 = 20_000; // 50 Hz
const MAG_INTERVAL_US: u64 = 50_000; // 20 Hz
const SONAR_INTERVAL_US: u64 = 100_000; // 10 Hz
const BATTERY_INTERVAL_US: u64 = 1_000_000; // 1 Hz
const GNSS_INTERVAL_US: u64 = 200_000; // 5 Hz
const RC_INTERVAL_US: u64 = 50_000; // 20 Hz
const OUTPUT_RAW_INTERVAL_US: u64 = 50_000; // 20 Hz

// used for converting names of ParamValues ("id" during creation in params.cpp) to null-terminated characters
pub const fn str_to_fixed_bytes(input: &str) -> [u8; 16] {
    let mut buffer = [0u8; 16];
    let input_bytes = input.as_bytes();

    // Determine how many bytes to copy (at most 16)
    let len_to_copy = if input_bytes.len() > 16 {
        16
    } else {
        input_bytes.len()
    };

    // Copy the bytes from the input string
    let mut i = 0;
    while i < len_to_copy {
        buffer[i] = input_bytes[i];
        i += 1;
    }

    // If the input was shorter than 16, the spot after the last character
    // is already a 0 from the initial buffer creation, so it is null-terminated.
    // If the input was 16 or longer, the buffer is full and not null-terminated.

    buffer
}

fn calibration_command_is_complete(command: RosflightCmd, flags: CalibrationFlags) -> bool {
    match command {
        RosflightCmd::AccelCalibration => !flags.contains(CalibrationFlags::ACCEL),
        RosflightCmd::GyroCalibration => !flags.contains(CalibrationFlags::GYRO),
        RosflightCmd::BaroCalibration => !flags.contains(CalibrationFlags::BARO),
        RosflightCmd::AirspeedCalibration => !flags.contains(CalibrationFlags::PITOT),
        _ => false,
    }
}

pub struct CommManager<B, T>
where
    B: board::BoardIo,
    T: comm_link_trait::CommInterface<B>,
{
    last_heartbeat_us: u64,
    last_status_send_us: u64,
    last_imu_send_us: u64,
    last_attitude_send_us: u64,
    last_baro_send_us: u64,
    last_mag_send_us: u64,
    last_sonar_send_us: u64,
    last_battery_send_us: u64,
    last_gnss_send_us: u64,
    last_rc_send_us: u64,
    last_output_raw_us: u64,

    pub sysid: u8,
    comm_link: T,
    pub msgs: comm_messages::Messages,
    pending_calibration_ack: Option<RosflightCmd>,
    _board_marker: PhantomData<B>,
}

impl<B, T> CommManager<B, T>
where
    B: board::BoardIo,
    T: comm_link_trait::CommInterface<B>,
{
    pub fn new(comm_link: T, now_us: u64) -> Self {
        CommManager {
            last_heartbeat_us: now_us,
            last_status_send_us: now_us,
            last_imu_send_us: now_us,
            last_attitude_send_us: now_us,
            last_baro_send_us: now_us,
            last_mag_send_us: now_us,
            last_sonar_send_us: now_us,
            last_battery_send_us: now_us,
            last_gnss_send_us: now_us,
            last_rc_send_us: now_us,
            last_output_raw_us: now_us,

            sysid: 0,
            comm_link,
            msgs: comm_messages::Messages::default(),
            pending_calibration_ack: None,
            _board_marker: PhantomData,
        }
    }

    pub fn set_pending_calibration_ack(&mut self, command: Option<RosflightCmd>) {
        self.pending_calibration_ack = command;
    }

    #[cfg(test)]
    pub(crate) fn comm_link(&self) -> &T {
        &self.comm_link
    }

    pub fn process_incoming_messages(&mut self, board: &mut B) {
        self.comm_link
            .handle_incoming_messages(board, &mut self.msgs);
    }

    pub fn send_named_telemetry_streams<S, A>(
        &mut self,
        board: &mut B,
        now_us: u64,
        state_manager: &StateManager,
        command_manager: &CommandManager,
        estimator_state: &S,
        processed_sensors: &ProcessedSensors,
        actuator_commands: &A,
    ) where
        S: AttitudeStateTrait,
        A: AsRef<[f64]>,
    {
        if now_us >= self.last_heartbeat_us + HEARTBEAT_INTERVAL_US {
            self.send_rosflight_heartbeat(
                board,
                HeartbeatMsg {
                    autopilot: 0,
                    base_mode: 0,
                    custom_mode: 0,
                    mavlink_version: 0,
                    system_status: 0,
                    type_: 0,
                },
            );
            self.last_heartbeat_us = now_us;
        }

        if now_us >= self.last_status_send_us + STATUS_INTERVAL_US {
            self.send_rosflight_status(
                board,
                RosflightStatusMsg {
                    armed: state_manager.is_armed() as u8,
                    failsafe: state_manager.is_in_failsafe() as u8,
                    rc_override: command_manager.get_rc_override(),
                    offboard: command_manager.is_offboard_active() as u8,
                    error_code: state_manager.get_errors(),
                    control_mode: command_manager.get_control_mode().into(),
                    num_errors: state_manager.get_errors().bits().count_ones() as i16,
                    loop_time_us: 0,
                },
            );
            self.last_status_send_us = now_us;
        }

        if let Some(imu_packet) = processed_sensors.imu {
            self.send_rosflight_small_imu(
                board,
                SmallImuMsg {
                    temperature: 0.0,
                    time_boot_us: imu_packet.header.timestamp,
                    xacc: imu_packet.accel[0] as f32,
                    yacc: imu_packet.accel[1] as f32,
                    zacc: imu_packet.accel[2] as f32,
                    xgyro: imu_packet.gyro[0] as f32,
                    ygyro: imu_packet.gyro[1] as f32,
                    zgyro: imu_packet.gyro[2] as f32,
                },
            );

            let q = estimator_state.q();
            let qd = estimator_state.q_dot();
            let rollspeed = 2.0 * (q[0] * qd[1] - q[1] * qd[0] - q[2] * qd[3] + q[3] * qd[2]);
            let pitchspeed = 2.0 * (q[0] * qd[2] - q[1] * qd[3] - q[2] * qd[0] + q[3] * qd[1]);
            let yawspeed = 2.0 * (q[0] * qd[3] - q[1] * qd[2] - q[2] * qd[1] + q[3] * qd[0]);

            self.send_rosflight_attitude_quaternion(
                board,
                AttitudeQuaternionMsg {
                    time_boot_ms: (imu_packet.header.timestamp / 1000) as u32,
                    q1: q[0],
                    q2: q[1],
                    q3: q[2],
                    q4: q[3],
                    rollspeed,
                    pitchspeed,
                    yawspeed,
                },
            );
        }

        if let Some(packet) = processed_sensors.baro {
            self.send_rosflight_small_baro(
                board,
                SmallBaroMsg {
                    altitude: 0.0,
                    pressure: packet.pressure,
                    temperature: packet.temperature,
                },
            );
        }

        if let Some(packet) = processed_sensors.mag {
            self.send_rosflight_small_mag(
                board,
                SmallMagMsg {
                    xmag: packet.flux[0],
                    ymag: packet.flux[1],
                    zmag: packet.flux[2],
                },
            );
        }

        if let Some(packet) = processed_sensors.range {
            self.send_rosflight_small_range(
                board,
                SmallRangeMsg {
                    type_: RosflightRangeType::RosflightRangeSonar,
                    range: packet.range,
                    max_range: 0.0,
                    min_range: 0.0,
                },
            );
        }

        if let Some(packet) = processed_sensors.battery {
            self.send_rosflight_battery_status(
                board,
                BatteryStatusMsg {
                    battery_voltage: packet.voltage,
                    battery_current: packet.current,
                },
            );
        }

        if let Some(packet) = processed_sensors.gnss {
            self.send_rosflight_gnss(
                board,
                RosflightGnssMsg {
                    rosflight_timestamp: packet.header.timestamp,
                    seconds: packet.sec as u64,
                    nanos: packet.nano as u32,
                    fix_type: packet.fix_type,
                    num_sat: packet.num_sats,
                    lat: packet.lat,
                    lon: packet.lon,
                    height: packet.height,
                    vel_n: packet.vel_n,
                    vel_e: packet.vel_e,
                    vel_d: packet.vel_d,
                    s_acc: packet.s_acc,
                    h_acc: packet.h_acc,
                    v_acc: packet.v_acc,
                },
            );
        }

        if let Some(packet) = processed_sensors.rc {
            let mut channels = [0u16; RC_PACKET_CHANNELS];
            let count = (packet.n_chan as usize).min(8).min(RC_PACKET_CHANNELS);
            for (dst, src) in channels.iter_mut().zip(packet.chan.iter()).take(count) {
                *dst = (*src * 1000.0 + 1000.0) as u16;
            }

            self.send_rosflight_rc_channels(
                board,
                RcChannelsMsg {
                    time_boot_ms: board.clock_millis(),
                    chancount: 0,
                    channels,
                    rssi: 0,
                },
            );
        }

        let mut values = [0.0f32; 14];
        for (dst, src) in values.iter_mut().zip(actuator_commands.as_ref().iter()) {
            *dst = *src as f32;
        }
        self.send_rosflight_output_raw(board, RosflightOutputRawMsg { stamp: now_us, values });
    }

    pub fn queue_completed_calibration_ack(
        &mut self,
        comm_events: &mut CommEventQueues,
        flags: CalibrationFlags,
    ) -> bool {
        let Some(command) = self.pending_calibration_ack else {
            return false;
        };

        if calibration_command_is_complete(command, flags) {
            if comm_events
                .responses
                .push(CommResponse::CmdAck(RosflightCmdAckMsg {
                    command,
                    success: RosflightCmdResponse::RosflightCmdSuccess,
                }))
                .is_err()
            {
                return false;
            }
            self.pending_calibration_ack = None;
            true
        } else {
            false
        }
    }

    pub fn act_on_messages(
        &mut self,
        param_events: &mut ParamEventQueues,
        comm_events: &mut CommEventQueues,
        command_events: &mut CommandEventQueues,
        companion_events: &mut CompanionEventQueues,
        board: &mut B,
    ) {
        if let Some(msg) = self.msgs.heartbeat.take() {
            let _ = companion_events
                .heartbeats
                .push(CompanionHeartbeatReceived { msg });
        }

        // first check the param_request_list
        if let Some(msg) = self.msgs.param_request_read.take() {
            let _ = param_events.read_requests.push(ParamReadRequested {
                identifier: msg.param_identifier,
            });
        }

        if self.msgs.param_request_list.take().is_some() {
            let _ = param_events.list_requests.push(ParamListRequested);
        }

        // next check for timesync messages
        let msg_opt: Option<TimesyncMsg> = self.msgs.timesync.take();
        if let Some(mut msg) = msg_opt {
            // fill ts1 (which is currently set to 0) and pass back to the companion computer immediately
            msg.ts1 = (board.clock_micros() * 1000) as i64;
            self.send_timesync(board, msg);
        }

        if let Some(msg) = self.msgs.offboard_control.take() {
            let now_us = board.clock_micros();
            let _ = command_events
                .offboard_control_requests
                .push(OffboardControlRequested { now_us, msg });
        }

        if let Some(msg) = self.msgs.aux_cmd.take() {
            let _ = companion_events
                .aux_commands
                .push(AuxCommandReceived { msg });
        }

        if let Some(msg) = self.msgs.external_attitude.take() {
            let _ = companion_events
                .external_attitudes
                .push(ExternalAttitudeReceived { msg });
        }

        if let Some(msg) = self.msgs.param_set.take() {
            let _ = param_events.set_requests.push(ParamSetRequested {
                value: msg.param_value,
                param_id_bytes: msg.param_id,
            });
        }

        // now act on ROSflight Commands

        let cmd_msg_opt = self.msgs.cmd.take();
        if let Some(msg) = cmd_msg_opt {
            // println!("Processing ROSflight command: {:?}", msg.command);
            //defmt::info!("Processing ROSflight command.");

            // Assume failure unless explicitly set to success
            let mut success = RosflightCmdResponse::RosflightCmdFailed;
            let mut send_ack_now = true;

            match msg.command {
                RosflightCmd::RcCalibration => {
                    if command_events
                        .rc_trim_calibration_requests
                        .push(RcTrimCalibrationRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::AccelCalibration => {
                    //defmt::info!("Starting Accelerometer Calibration.");
                    if command_events
                        .calibration_requests
                        .push(CalibrationRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::GyroCalibration => {
                    //defmt::info!("Starting Gyro Calibration.");
                    if command_events
                        .calibration_requests
                        .push(CalibrationRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::BaroCalibration => {
                    //defmt::info!("Starting Baro Calibration.");
                    if command_events
                        .calibration_requests
                        .push(CalibrationRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::AirspeedCalibration => {
                    //defmt::info!("Starting Airspeed Calibration.");
                    if command_events
                        .calibration_requests
                        .push(CalibrationRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::ReadParams => {
                    if command_events
                        .board_command_requests
                        .push(BoardCommandRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::WriteParams => {
                    if command_events
                        .board_command_requests
                        .push(BoardCommandRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::SetParamDefaults => {
                    //defmt::info!("Setting parameters to defaults.");
                    if command_events
                        .param_defaults_requests
                        .push(ParamDefaultsRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::Reboot => {
                    if command_events
                        .board_command_requests
                        .push(BoardCommandRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::RebootToBootloader => {
                    if command_events
                        .board_command_requests
                        .push(BoardCommandRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::SendVersion => {
                    if command_events
                        .version_requests
                        .push(VersionRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::ResetOrigin => {
                    if command_events
                        .reset_origin_requests
                        .push(ResetOriginRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::SendAllConfigInfos => {
                    if command_events
                        .config_info_requests
                        .push(ConfigInfoRequested {
                            command: msg.command,
                        })
                        .is_ok()
                    {
                        send_ack_now = false;
                    }
                }
            } // end match

            if send_ack_now {
                let ack_msg = RosflightCmdAckMsg {
                    command: msg.command,
                    success,
                };
                let _ = comm_events.responses.push(CommResponse::CmdAck(ack_msg));
            }
        } // end if let Some(msg)

    }

    pub fn send_comm_responses(
        &mut self,
        board: &mut B,
        comm_events: &mut CommEventQueues,
    ) {
        while let Some(response) = comm_events.responses.pop() {
            match response {
                CommResponse::ParamValue(msg) => {
                    if msg.param_index == ParamId::PARAM_SYSTEM_ID as u16 {
                        if let ParamValue::Int(new_sysid) = msg.param_value {
                            self.sysid = new_sysid as u8;
                        }
                    }
                    self.comm_link.send_named_value(board, self.sysid, msg);
                }
                CommResponse::CmdAck(msg) => {
                    self.comm_link.send_cmd_ack(board, self.sysid, msg);
                }
                CommResponse::Version(msg) => {
                    self.comm_link.send_version(board, self.sysid, msg);
                }
                CommResponse::Statustext(msg) => {
                    self.comm_link.send_statustext(board, self.sysid, msg);
                }
            }
        }
    }

    pub fn send_timesync(&mut self, board: &mut B, msg: TimesyncMsg) {
        self.comm_link.send_timesync(board, self.sysid, msg);
    }

    pub fn send_rosflight_heartbeat(&mut self, board: &mut B, msg: HeartbeatMsg) {
        self.comm_link.send_heartbeat(board, self.sysid, msg);
    }

    pub fn send_rosflight_status(&mut self, board: &mut B, msg: RosflightStatusMsg) {
        self.comm_link.send_status(board, self.sysid, msg);
    }

    pub fn send_rosflight_attitude_quaternion(
        &mut self,
        board: &mut B,
        msg: AttitudeQuaternionMsg,
    ) {
        self.comm_link.send_attitude(board, self.sysid, msg);
    }

    pub fn send_rosflight_small_imu(&mut self, board: &mut B, msg: SmallImuMsg) {
        self.comm_link.send_imu(board, self.sysid, msg);
    }

    pub fn send_rosflight_small_baro(&mut self, board: &mut B, msg: SmallBaroMsg) {
        self.comm_link.send_baro(board, self.sysid, msg);
    }

    pub fn send_rosflight_small_mag(&mut self, board: &mut B, msg: SmallMagMsg) {
        self.comm_link.send_mag(board, self.sysid, msg);
    }

    pub fn send_rosflight_small_range(&mut self, board: &mut B, msg: SmallRangeMsg) {
        self.comm_link.send_range(board, self.sysid, msg);
    }

    pub fn send_rosflight_battery_status(&mut self, board: &mut B, msg: BatteryStatusMsg) {
        self.comm_link.send_battery_status(board, self.sysid, msg);
    }

    pub fn send_rosflight_gnss(&mut self, board: &mut B, msg: RosflightGnssMsg) {
        self.comm_link.send_gnss(board, self.sysid, msg);
    }

    pub fn send_rosflight_rc_channels(&mut self, board: &mut B, msg: RcChannelsMsg) {
        self.comm_link.send_rc_channels(board, self.sysid, msg);
    }

    pub fn send_rosflight_output_raw(&mut self, board: &mut B, msg: RosflightOutputRawMsg) {
        self.comm_link.send_output_raw(board, self.sysid, msg);
    }

}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        board::BoardIo,
        command_manager::CommandManager,
        command_system::{self, CalibrationRequestCtx},
        comm_messages::{
            enums::{
                OffboardControlIgnore, OffboardControlMode, ParamIdentifier, RosflightAuxCmdType,
                RosflightCmd, RosflightCmdResponse,
            },
            messages::{
                ExternalAttitudeMsg, HeartbeatMsg, OffboardControlMsg, ParamRequestReadMsg,
                ParamSetMsg, RosflightAuxCmdMsg, RosflightCmdMsg,
            },
        },
        events::{
            CommEventQueues, CommandEventQueues, CommResponse, CompanionEventQueues,
            ParamEventQueues,
        },
        param_system::{self, ParamApplyCtx, ParamListCtx, ParamListState, ParamReadCtx},
        params2::{ParamId, ParamValue, Params},
        ports::{EventDrainPort, EventEmitPort, ParamsReadPort, ParamsWritePort},
        sensorprocessors::CalibrationFlags,
        sensors::ProcessedSensors,
        state_machine::{Event, StateManager},
        test_support::{RecordingCommLink, TestBoard},
    };

    fn initialized_state() -> StateManager {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state
    }

    fn companion_events() -> CompanionEventQueues {
        CompanionEventQueues::default()
    }

    #[test]
    fn param_set_emits_request_without_mutating_or_acknowledging() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut params = Params::new();
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.param_set = Some(ParamSetMsg {
            target_system: 1,
            target_component: 1,
            param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
            param_value: ParamValue::Int(42),
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert_eq!(manager.comm_link.sent_param_value_count, 0);

        let request = param_events.set_requests.pop().unwrap();
        assert_eq!(request.value, ParamValue::Int(42));
        assert_eq!(request.param_id_bytes, *b"SYS_ID\0\0\0\0\0\0\0\0\0\0");
    }

    #[test]
    fn param_request_list_emits_request_without_streaming_from_comms() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let params = Params::new();
        let mut param_list_state = ParamListState::default();
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.param_request_list = Some(ParamRequestListMsg {
            target_system: 1,
            target_component: 1,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link.sent_param_value_count, 0);
        assert_eq!(param_events.list_requests.len(), 1);

        param_system::service_param_list_requests(ParamListCtx {
            params: ParamsReadPort::new(&params),
            state: &mut param_list_state,
            requests: EventDrainPort::new(&mut param_events.list_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
        });

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.comm_link.sent_param_value_count, 1);
        let sent = manager.comm_link.sent_param_values[0].unwrap();
        assert_eq!(sent.param_index, ParamId::PARAM_BAUD_RATE as u16);
        assert_eq!(sent.param_value, ParamValue::Int(921600));
    }

    #[test]
    fn param_request_read_emits_request_without_reading_from_comms() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.param_request_read = Some(ParamRequestReadMsg {
            target_system: 1,
            target_component: 1,
            param_identifier: ParamIdentifier::ID(*b"SYS_ID\0\0\0\0\0\0\0\0\0\0"),
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link.sent_param_value_count, 0);
        assert_eq!(param_events.read_requests.len(), 1);

        param_system::service_param_read_requests(ParamReadCtx {
            params: ParamsReadPort::new(&params),
            requests: EventDrainPort::new(&mut param_events.read_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
        });

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.comm_link.sent_param_value_count, 1);
        let sent = manager.comm_link.sent_param_values[0].unwrap();
        assert_eq!(sent.param_index, ParamId::PARAM_SYSTEM_ID as u16);
        assert_eq!(sent.param_value, ParamValue::Int(42));
    }

    #[test]
    fn send_comm_responses_sends_param_value_and_updates_sysid() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut comm_events = CommEventQueues::default();

        let _ = comm_events
            .responses
            .push(CommResponse::ParamValue(ParamValueMsg {
                param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
                param_value: ParamValue::Int(42),
                param_count: 1,
                param_index: ParamId::PARAM_SYSTEM_ID as u16,
            }));

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.sysid, 42);
        assert_eq!(manager.comm_link.sent_param_value_count, 1);

        let sent = manager.comm_link.sent_param_values[0].unwrap();
        assert_eq!(sent.param_id, *b"SYS_ID\0\0\0\0\0\0\0\0\0\0");
        assert_eq!(sent.param_value, ParamValue::Int(42));
        assert!(comm_events.responses.is_empty());
    }

    #[test]
    fn send_comm_responses_sends_command_ack_and_version() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut comm_events = CommEventQueues::default();

        let _ = comm_events
            .responses
            .push(CommResponse::Version(RosflightVersionMsg {
                version: [7; 50],
            }));
        let _ = comm_events
            .responses
            .push(CommResponse::CmdAck(RosflightCmdAckMsg {
                command: RosflightCmd::SendVersion,
                success: RosflightCmdResponse::RosflightCmdSuccess,
            }));
        let _ = comm_events
            .responses
            .push(CommResponse::Statustext(StatustextMsg {
                severity: Severity::Info,
                text: [9; 50],
            }));

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.comm_link().version_count, 1);
        assert_eq!(manager.comm_link().last_version.unwrap().version, [7; 50]);
        assert_eq!(manager.comm_link().cmd_ack_count, 1);
        assert_eq!(manager.comm_link().statustext_count, 1);
        assert_eq!(manager.comm_link().last_statustext.unwrap().text, [9; 50]);
        assert!(matches!(
            manager.comm_link().last_cmd_ack.unwrap().command,
            RosflightCmd::SendVersion
        ));
        assert!(comm_events.responses.is_empty());
    }

    #[test]
    fn send_version_command_enqueues_version_and_ack_responses() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SendVersion,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().version_count, 0);
        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());
        assert_eq!(command_events.version_requests.len(), 1);

        command_system::apply_version_requests(command_system::VersionRequestCtx {
            requests: EventDrainPort::new(&mut command_events.version_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
            state: &initialized_state(),
        });
        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.comm_link().version_count, 1);
        assert_eq!(manager.comm_link().cmd_ack_count, 1);
        assert!(matches!(
            manager.comm_link().last_cmd_ack.unwrap().success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn param_set_pipeline_defers_ack_until_after_apply_stage() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut params = Params::new();
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.param_set = Some(ParamSetMsg {
            target_system: 1,
            target_component: 1,
            param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
            param_value: ParamValue::Int(42),
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert_eq!(manager.comm_link.sent_param_value_count, 0);

        param_system::apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut params),
            requests: EventDrainPort::new(&mut param_events.set_requests),
            changes: EventEmitPort::new(&mut param_events.changes),
            responses: EventEmitPort::new(&mut comm_events.responses),
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(manager.comm_link.sent_param_value_count, 0);

        let change = param_events.changes.iter().next().unwrap();
        assert_eq!(change.id, ParamId::PARAM_SYSTEM_ID);
        assert_eq!(change.old, ParamValue::Int(1));
        assert_eq!(change.new, ParamValue::Int(42));

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.sysid, 42);
        assert_eq!(manager.comm_link.sent_param_value_count, 1);
        assert!(comm_events.responses.is_empty());
    }

    #[test]
    fn named_telemetry_sends_sensor_state_and_output_messages() {
        let mut board = TestBoard {
            current_time_us: 1_100_000,
            tx_write_count: 0,
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), 0);
        let state_manager = StateManager::new();
        let command_manager = CommandManager::new();
        let estimator_state = crate::estimator::quad_estimator::AttitudeState::default();
        let actuator_commands = [0.1, 0.2, 0.3, 0.4];
        let mut processed_sensors = ProcessedSensors::default();
        processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 9_000,
                status: 0,
            },
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            temperature: 25.0,
            seq: 1,
        });

        let now_us = board.clock_micros();

        manager.send_named_telemetry_streams(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
        );

        assert_eq!(manager.comm_link().heartbeat_count, 1);
        assert_eq!(manager.comm_link().status_count, 1);
        assert_eq!(manager.comm_link().imu_count, 1);
        assert_eq!(manager.comm_link().attitude_count, 1);
        assert_eq!(manager.comm_link().output_raw_count, 1);

        let output = manager.comm_link().last_output_raw.unwrap();
        assert_eq!(output.stamp, 1_100_000);
        assert_eq!(output.values[0], 0.1);
        assert_eq!(output.values[1], 0.2);
        assert_eq!(output.values[2], 0.3);
        assert_eq!(output.values[3], 0.4);
    }

    #[test]
    fn named_rc_telemetry_matches_upstream_raw_channel_packing() {
        let mut board = TestBoard {
            current_time_us: 1_234_000,
            tx_write_count: 0,
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), 0);
        let state_manager = StateManager::new();
        let command_manager = CommandManager::new();
        let estimator_state = crate::estimator::quad_estimator::AttitudeState::default();
        let mut processed_sensors = ProcessedSensors::default();
        let mut rc_packet = crate::packets::RcPacket::default();
        rc_packet.n_chan = RC_PACKET_CHANNELS as u32;
        let test_channels = [
            -1.0, -0.5, 0.0, 0.5, 1.0, 0.25, -0.25, 0.75, 0.33, 0.44, 0.55, 0.66, 0.77,
            0.88, 0.99, -0.99, 1.0, -1.0,
        ];
        rc_packet.chan[..test_channels.len()].copy_from_slice(&test_channels);
        processed_sensors.rc = Some(rc_packet);
        let now_us = board.clock_micros();

        manager.send_named_telemetry_streams(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &estimator_state,
            &processed_sensors,
            &[0.0; 4],
        );

        let msg = manager.comm_link().last_rc_channels.unwrap();
        assert_eq!(manager.comm_link().rc_channels_count, 1);
        assert_eq!(msg.time_boot_ms, 1234);
        assert_eq!(msg.chancount, 0);
        assert_eq!(msg.rssi, 0);
        assert_eq!(
            &msg.channels[..8],
            &[0, 500, 1000, 1500, 2000, 1250, 750, 1750]
        );
        assert!(msg.channels[8..].iter().all(|channel| *channel == 0));
    }

    #[test]
    fn named_status_telemetry_reports_command_manager_override_state() {
        let mut board = TestBoard {
            current_time_us: 1_100_000,
            tx_write_count: 0,
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), 0);
        let state_manager = StateManager::new();
        let mut command_manager = CommandManager::new();
        let params = Params::new();
        let estimator_state = crate::estimator::quad_estimator::AttitudeState::default();
        let processed_sensors = ProcessedSensors::default();
        let actuator_commands = [0.0, 0.0, 0.0, 0.0];

        command_manager.set_new_offboard_command(
            board.clock_micros(),
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: 0.0,
            },
            &params,
        );

        manager.send_named_telemetry_streams(
            &mut board,
            1_100_000,
            &state_manager,
            &command_manager,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
        );

        let status = manager.comm_link().last_status.unwrap();
        assert_eq!(status.offboard, 1);
        assert_eq!(status.rc_override, 0);
    }

    #[test]
    fn calibration_command_ack_is_deferred_until_flag_clears() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();
        let mut cal_flags = CalibrationFlags::empty();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::GyroCalibration,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert!(cal_flags.is_empty());
        let started = command_system::apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut command_events.calibration_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
            state: &initialized_state(),
            flags: &mut cal_flags,
        });
        manager.set_pending_calibration_ack(started);

        assert!(cal_flags.contains(CalibrationFlags::GYRO));
        assert_eq!(manager.comm_link().cmd_ack_count, 0);

        cal_flags.remove(CalibrationFlags::GYRO);

        assert!(manager.queue_completed_calibration_ack(&mut comm_events, cal_flags));
        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        manager.send_comm_responses(&mut board, &mut comm_events);
        assert_eq!(manager.comm_link().cmd_ack_count, 1);

        let ack = manager.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::GyroCalibration));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn offboard_control_message_emits_command_event() {
        let mut board = TestBoard {
            current_time_us: 55_000,
            tx_write_count: 0,
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.offboard_control = Some(OffboardControlMsg {
            mode: OffboardControlMode::ModeRollPitchYawrateThrottle,
            ignore: OffboardControlIgnore::IGNORE_FY,
            qx: 0.1,
            qy: 0.2,
            qz: 0.3,
            fx: 0.4,
            fy: 0.5,
            fz: 0.6,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        let request = command_events.offboard_control_requests.pop().unwrap();
        assert_eq!(request.now_us, 55_000);
        assert_eq!(request.msg.mode, OffboardControlMode::ModeRollPitchYawrateThrottle);
        assert!(request.msg.ignore.contains(OffboardControlIgnore::IGNORE_FY));
        assert_eq!(request.msg.qx, 0.1);
    }

    #[test]
    fn companion_inputs_emit_companion_events() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();
        let mut companion_events = CompanionEventQueues::default();

        manager.msgs.heartbeat = Some(HeartbeatMsg {
            type_: 1,
            autopilot: 2,
            base_mode: 3,
            custom_mode: 4,
            system_status: 5,
            mavlink_version: 6,
        });
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; 14],
            aux_cmd_array: [0.0; 14],
        };
        aux.type_array[1] = RosflightAuxCmdType::Motor;
        aux.aux_cmd_array[1] = 0.4;
        manager.msgs.aux_cmd = Some(aux);
        manager.msgs.external_attitude = Some(ExternalAttitudeMsg {
            qw: 1.0,
            qx: 0.1,
            qy: 0.2,
            qz: 0.3,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events,
            &mut board,
        );

        assert_eq!(companion_events.heartbeats.len(), 1);
        assert_eq!(companion_events.aux_commands.len(), 1);
        assert_eq!(companion_events.external_attitudes.len(), 1);
        assert_eq!(
            companion_events.heartbeats.pop().unwrap().msg.system_status,
            5
        );
        let aux_event = companion_events.aux_commands.pop().unwrap();
        assert!(matches!(
            aux_event.msg.type_array[1],
            RosflightAuxCmdType::Motor
        ));
        assert_eq!(aux_event.msg.aux_cmd_array[1], 0.4);
        assert_eq!(
            companion_events
                .external_attitudes
                .pop()
                .unwrap()
                .msg
                .qz,
            0.3
        );
    }

    #[test]
    fn set_param_defaults_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SetParamDefaults,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(manager.comm_link().cmd_ack_count, 0);

        command_system::apply_param_defaults_requests(command_system::ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut command_events.param_defaults_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
            state: &initialized_state(),
            params: &mut params,
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        manager.send_comm_responses(&mut board, &mut comm_events);
        assert_eq!(manager.comm_link().cmd_ack_count, 1);

        let ack = manager.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn board_command_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::WriteParams,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());

        let request = command_events.board_command_requests.pop().unwrap();
        assert!(matches!(request.command, RosflightCmd::WriteParams));
    }

    #[test]
    fn rc_trim_calibration_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::RcCalibration,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());

        let request = command_events.rc_trim_calibration_requests.pop().unwrap();
        assert!(matches!(request.command, RosflightCmd::RcCalibration));
    }

    #[test]
    fn reset_origin_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::ResetOrigin,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());

        let request = command_events.reset_origin_requests.pop().unwrap();
        assert!(matches!(request.command, RosflightCmd::ResetOrigin));
    }

    #[test]
    fn send_all_config_infos_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SendAllConfigInfos,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());

        let request = command_events.config_info_requests.pop().unwrap();
        assert!(matches!(request.command, RosflightCmd::SendAllConfigInfos));
    }
}
