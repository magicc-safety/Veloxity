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
use crate::mavlink::dialects::Rosflight;
use crate::params2::{ParamDefinition, ParamId, ParamValue, PARAMS_COUNT, ParamIter, Params, PARAM_DEFINITIONS};
use crate::sensorprocessors::CalibrationFlags;
use core::marker::PhantomData;

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

pub struct CommManager<B, T>
where
    B: board::BoardTrait,
    T: comm_link_trait::CommInterface<B>,
{
    pub sysid: u8,
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

    pub fn act_on_messages(&mut self, params_iter: &mut Option<ParamIter>, params: &mut Params, cal_flags: &mut CalibrationFlags, board: &mut B) {

        // first check the param_request_list
        if self.msgs.param_request_list.take().is_some() {
            if params_iter.is_none() {
                *params_iter = Some(params.iter());
            }
        }

        // If we're in the middle of sending the parameters up, we're still "handling" that message
        if let Some(iterator) = params_iter {

            // Safely get the next item. This `if let` replaces your `.unwrap()`.
            if let Some((param_id, param_val)) = iterator.next() {
                let def = &PARAM_DEFINITIONS[param_id as usize];
        
                // You now have everything you need to send the message:
                // def.name    -> The parameter's string name (e.g., "SYS_ID")
                // param_id    -> The enum ID (e.g., ParamId::PARAM_SYSTEM_ID)
                // param_val   -> The current value (e.g., ParamValue::Int(1))
                self.send_param_value(def, param_val, board);

            } else {
                // The iterator is finished, so set it back to None.
                // This is crucial for preventing future panics and resetting the state.
                *params_iter = None;
            }
        }

        // next check for timesync messages
        let msg_opt: Option<TimesyncMsg> = self.msgs.timesync.take();
        if let Some(mut msg) = msg_opt {
            // fill ts1 (which is currently set to 0) and pass back to the companion computer immediately
            msg.ts1 = (board.clock_micros() * 1000) as i64;
            self.send_timesync(board, msg);
        }

        // now check for parameter set requests
        // let msg_opt = self.msgs.param_set.take();
        // if let Some(mut msg) = msg_opt {
        //     // TODO will have to add checking on target system and component system before matching here...
        //     params.set_by_name(msg.param_id, msg.param_value);
        // }

        let msg_opt = self.msgs.param_set.take();
        if let Some(msg) = msg_opt { // No need for `mut` if you only read from msg

            // TODO: Add checking on target system and component ID here if needed
            // if msg.target_system != self.sysid || msg.target_component != self.component_id {
            //     return; // Or log an error, ignore message, etc.
            // }

            // Convert the incoming [u8; 16] param_id to a &str
            let param_name_bytes = &msg.param_id;
            // Find the position of the first null byte, or take the full length
            let len = param_name_bytes.iter().position(|&b| b == 0).unwrap_or(param_name_bytes.len());
            let name_slice = &param_name_bytes[..len];

            // Attempt to convert the byte slice to a UTF-8 &str
            match core::str::from_utf8(name_slice) {
                Ok(param_name_str) => {
                    // Successfully converted, now set the parameter
                    if params.set_by_name(param_name_str, msg.param_value) {
                        defmt::info!("Set parameter '{}' successfully.", param_name_str);

                        // if the param was the system id, update the comm_manager's systemid


                        // MAVLink spec requires acknowledging the change by sending PARAM_VALUE
                        // Find the ParamDefinition to get the ID and count
                        if let Some(def) = PARAM_DEFINITIONS.iter().find(|d| d.name == param_name_str) {
                            let value_msg = ParamValueMsg {
                                param_id: msg.param_id, // Use the received ID bytes
                                param_value: msg.param_value, // Use the value that was set
                                param_count: PARAMS_COUNT as u16,
                                param_index: def.id as u16,
                            };
                            // Assuming 'self.comm_link' and 'board' are accessible
                            // Need system ID - retrieve from params or store in CommManager
                            self.comm_link.send_named_value(board, self.sysid, value_msg);
                        } else {
                            defmt::info!("Error: Could not find definition for '{}' after setting.", param_name_str);
                        }

                    } else {
                        defmt::info!("Failed to set parameter: Name '{}' not found.", param_name_str);
                        // Optionally send a NACK or STATUSTEXT message here
                    }
                }
                Err(e) => {
                    // The received param_id was not valid UTF-8
                    defmt::info!("Received PARAM_SET with invalid UTF-8 name: {:?}", name_slice);
                }
            }
        }

        // now act on ROSflight Commands

        let cmd_msg_opt = self.msgs.cmd.take();
        if let Some(msg) = cmd_msg_opt {
            // println!("Processing ROSflight command: {:?}", msg.command);
            defmt::info!("Processing ROSflight command.");

            // Assume failure unless explicitly set to success
            let mut success = RosflightCmdResponse::RosflightCmdFailed;

            match msg.command {
                RosflightCmd::RcCalibration => {
                    // Placeholder: Actual RC calibration logic would go here.
                    // This often involves reading min/max/trim values from the RC
                    // receiver over a period and storing them. This is complex
                    // and might need interaction with an `Rc` struct/module.
                    defmt::info!("Warning: RC Calibration not implemented.");
                    // success = RosflightCmdResponse::RosflightCmdSuccess; // Mark success if implemented
                }
                RosflightCmd::AccelCalibration => {
                    defmt::info!("Starting Accelerometer Calibration.");
                    cal_flags.insert(CalibrationFlags::ACCEL); // Set the flag
                    // The actual calibration happens over time in ImuProcessor
                    success = RosflightCmdResponse::RosflightCmdSuccess; // Acknowledge start
                }
                RosflightCmd::GyroCalibration => {
                    defmt::info!("Starting Gyro Calibration.");
                    cal_flags.insert(CalibrationFlags::GYRO); // Set the flag
                    // The actual calibration happens over time in ImuProcessor
                    success = RosflightCmdResponse::RosflightCmdSuccess; // Acknowledge start
                }
                RosflightCmd::BaroCalibration => {
                    defmt::info!("Starting Baro Calibration.");
                    cal_flags.insert(CalibrationFlags::BARO); // Set the flag
                    // The actual calibration happens over time in BaroProcessor
                    success = RosflightCmdResponse::RosflightCmdSuccess; // Acknowledge start
                }
                RosflightCmd::AirspeedCalibration => {
                    defmt::info!("Starting Airspeed Calibration.");
                    cal_flags.insert(CalibrationFlags::PITOT); // Set the flag
                    // The actual calibration happens over time in PitotProcessor
                    success = RosflightCmdResponse::RosflightCmdSuccess; // Acknowledge start
                }
                RosflightCmd::ReadParams => {
                    // Placeholder: Need BoardTrait method for reading from non-volatile memory
                    defmt::info!("Warning: ReadParams (from non-volatile) not implemented.");
                    // if board.read_params_from_memory(params) {
                    //     success = RosflightCmdResponse::RosflightCmdSuccess;
                    // }
                }
                RosflightCmd::WriteParams => {
                    // Placeholder: Need BoardTrait method for writing to non-volatile memory
                    defmt::info!("Warning: WriteParams (to non-volatile) not implemented.");
                    // if board.write_params_to_memory(params) {
                    //     success = RosflightCmdResponse::RosflightCmdSuccess;
                    // }
                }
                RosflightCmd::SetParamDefaults => {
                    defmt::info!("Setting parameters to defaults.");
                    params.set_defaults();
                    success = RosflightCmdResponse::RosflightCmdSuccess;
                }
                RosflightCmd::Reboot => {
                    // Placeholder: Need BoardTrait method for reboot
                    defmt::info!("Warning: Reboot command not implemented.");
                    // board.reboot();
                    // success = RosflightCmdResponse::RosflightCmdSuccess; // Won't actually send if reboot works!
                }
                RosflightCmd::RebootToBootloader => {
                    // Placeholder: Need BoardTrait method for rebooting to bootloader
                    defmt::info!("Warning: RebootToBootloader command not implemented.");
                    // board.reboot_to_bootloader();
                    // success = RosflightCmdResponse::RosflightCmdSuccess; // Won't actually send if reboot works!
                }
                RosflightCmd::SendVersion => {
                    // Placeholder: Define version somewhere (e.g., compile-time const)
                    defmt::info!("Sending Version Info (Not fully implemented)");
                    let version_str = "RustFlight Alpha 0.1"; // Example version string
                    let mut version_bytes = [0u8; 50];
                    let len = version_str.len().min(version_bytes.len());
                    version_bytes[..len].copy_from_slice(version_str.as_bytes());

                    let version_msg = RosflightVersionMsg { version: version_bytes };
                    self.comm_link.send_version(board, self.sysid, version_msg);
                    success = RosflightCmdResponse::RosflightCmdSuccess;
                }
                RosflightCmd::ResetOrigin => {
                    // Placeholder: Logic depends on your estimator implementation
                    defmt::info!("Warning: ResetOrigin command not implemented.");
                    // Call relevant function on your estimator instance if applicable
                    // success = RosflightCmdResponse::RosflightCmdSuccess;
                }
                RosflightCmd::SendAllConfigInfos => {
                    defmt::info!("Warning: SendAllConfigInfos command not implemented.");
                    // This is less common, might involve sending detailed setup info.
                    // success = RosflightCmdResponse::RosflightCmdSuccess;
                }
            } // end match

            // --- Send ROSFLIGHT_CMD_ACK ---
            let ack_msg = RosflightCmdAckMsg {
                command: msg.command, // Echo the command that was processed
                success: success,     // Indicate success or failure
            };
            // Need to add send_cmd_ack to CommInterface and MavlinkInterface
            self.comm_link.send_cmd_ack(board, self.sysid, ack_msg);
            // println!("Sent ACK for command {:?} with status {:?}", ack_msg.command, ack_msg.success);
            defmt::info!("Sent ACK")
        } // end if let Some(msg)
    }

    // SENDING PLACEHOLDER MESSAGES
    pub fn send_heartbeat(&mut self, board: &mut B, hb: HeartbeatMsg) {
        self.comm_link.send_heartbeat(board, self.sysid, hb);
    }

    pub fn send_timesync(&mut self, board: &mut B, ts: TimesyncMsg) {
        self.comm_link.send_timesync(board, self.sysid, ts);
    }

    pub fn send_status(&mut self, board: &mut B, sm: RosflightStatusMsg) {
        self.comm_link.send_status(board, self.sysid, sm);
    }

    // pub fn send_named_value(&mut self, board: &mut B) {
    //     let msg = ParamValueMsg {
    //         param_id: *b"TEST_PARAM_ID___",
    //         param_value: ParamValue::Float(123.45),
    //         param_count: 1,
    //         param_index: 0,
    //     };
    //     self.comm_link.send_named_value(board, self.sysid, msg);
    // }

    pub fn send_param_value(&mut self, def: &ParamDefinition, val: ParamValue, board: &mut B) {
        let msg = ParamValueMsg {
            param_id: str_to_fixed_bytes(def.name),
            param_value: val,
            param_count: PARAMS_COUNT as u16,
            param_index: def.id as u16,
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
