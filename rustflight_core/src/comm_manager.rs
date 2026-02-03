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

use crate::{board, log_info};
use crate::comm_messages::{self, messages::*, enums::*};
use crate::state_machine::StateManager;
use crate::command_manager::CommandManager;
use crate::estimator::{AttitudeStateTrait, Estimator};
use crate::mavlink::dialects::Rosflight;
use crate::packets::{self, RC_PACKET_CHANNELS};
use crate::hlist::*;
use crate::bodytype::BodyType;
use crate::rustflight::Configuration;
use crate::params2::{ParamDefinition, ParamId, ParamValue, PARAMS_COUNT, ParamIter, Params, PARAM_DEFINITIONS};
use crate::sensorprocessors::CalibrationFlags;
use core::marker::PhantomData;

const HEARTBEAT_INTERVAL_US: u64 = 1_000_000; // 1 second = 1,000,000 microseconds
const STATUS_INTERVAL_US: u64 = 500_000;    // 2 Hz
const ATTITUDE_INTERVAL_US: u64 = 10_000;   // 100 Hz
const IMU_INTERVAL_US: u64 = 2500;          // 400 Hz
const BARO_INTERVAL_US: u64 = 20_000;         // 50 Hz
const MAG_INTERVAL_US: u64 = 50_000;          // 20 Hz
const SONAR_INTERVAL_US: u64 = 100_000;       // 10 Hz
const BATTERY_INTERVAL_US: u64 = 1_000_000;   // 1 Hz
const GNSS_INTERVAL_US: u64 = 200_000;        // 5 Hz
const RC_INTERVAL_US: u64 = 50_000;           // 20 Hz
const OUTPUT_RAW_INTERVAL_US: u64 = 50_000;   // 20 Hz

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
    _board_marker: PhantomData<B>,
}

impl<B, T> CommManager<B, T>
where
    B: board::BoardTrait,
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
            _board_marker: PhantomData,
        }
    }

    pub fn send_telemetry_streams<BT, C, A>(
        &mut self,
        board: &mut B,
        now_us: u64,
        state_manager: &StateManager,
        command_manager: &CommandManager,
        params: &Params,
        estimator_state: &<BT::Estimator as Estimator>::State,
        processed_sensors: &B::ProcessedSensorSet,
        actuator_commands: &A,
    )
    where
        BT: BodyType,
        A: AsRef<[f64]>,
        C: Configuration<B, BT>,
        BT::Estimator: Estimator,
        <BT::Estimator as Estimator>::State: AttitudeStateTrait,
        B::ProcessedSensorSet: HListGet<Option<packets::ImuPacket>, C::ImuPacketIndex> +
            HListGet<Option<packets::MagPacket>, C::MagPacketIndex> +
            HListGet<Option<packets::BaroPacket>, C::BaroPacketIndex> +
            HListGet<Option<packets::PitotPacket>, C::PitotPacketIndex> +
            HListGet<Option<packets::RangePacket>, C::RangePacketIndex> +
            HListGet<Option<packets::GNSSPacket>, C::GNSSPacketIndex> +
            HListGet<Option<packets::BatteryPacket>, C::BatteryPacketIndex> +
            HListGet<Option<packets::AttitudePacket>, C::AttitudePacketIndex> +
            HListGet<Option<packets::RcPacket>, C::RcPacketIndex>
    {
        let now_ms = (now_us / 1000) as u32;

        // Handle Heartbeat Message
        if now_us >= self.last_heartbeat_us + HEARTBEAT_INTERVAL_US {

            let hb = HeartbeatMsg {
                autopilot: 0,
                base_mode: 0,
                custom_mode: 0,
                mavlink_version: 0,
                system_status: 0,
                type_: 0
            };
            self.send_rosflight_heartbeat(board, hb);
            self.last_heartbeat_us = now_us;
        }

        // Handle Status Message
        if now_us >= self.last_status_send_us + STATUS_INTERVAL_US {
            let status_msg = comm_messages::messages::RosflightStatusMsg {
                armed: state_manager.is_armed() as u8,
                failsafe: state_manager.is_in_failsafe() as u8,
                rc_override: 0, // Placeholder: self.command_manager.is_rc_override() as u8,
                offboard: 0, // Placeholder: self.command_manager.is_offboard() as u8,
                error_code: state_manager.get_errors(),
                control_mode: command_manager.get_control_mode().into(),
                num_errors: state_manager.get_errors().bits().count_ones() as i16,
                loop_time_us: 0, // Placeholder
            };
            // defmt::info!("Sending Status: armed={}, failsafe={}, errors=0x{:b}", 
                // status_msg.armed, status_msg.failsafe, status_msg.error_code.bits());
            self.send_rosflight_status(board, status_msg);
            self.last_status_send_us = now_us;
        }

        // --- Send IMU and Attitude Telemetry ---
        // Send immediately when called (matches C: send when got.imu flag is set)
        // No interval timer - this function is only called when there's new IMU data
        let imu_packet_option: &Option<packets::ImuPacket> =
            processed_sensors.get();

        if let Some(imu_packet) = imu_packet_option {
            // Send IMU message
            let imu_msg = comm_messages::messages::SmallImuMsg {
                temperature: 0.0f32,
                time_boot_us: imu_packet.header.timestamp,
                xacc: imu_packet.accel[0] as f32,
                yacc: imu_packet.accel[1] as f32,
                zacc: imu_packet.accel[2] as f32,
                xgyro: imu_packet.gyro[0] as f32,
                ygyro: imu_packet.gyro[1] as f32,
                zgyro: imu_packet.gyro[2] as f32,
            };
            self.send_rosflight_small_imu(board, imu_msg);

            // Send attitude message (with every IMU, matching C implementation)
            // Use IMU timestamp directly from IMU packet (matches C: uses sensor timestamp)
            let q = estimator_state.q();    // [w, x, y, z]
            let qd = estimator_state.q_dot(); // [w_dot, x_dot, y_dot, z_dot]

            let w = q[0]; let x = q[1]; let y = q[2]; let z = q[3];
            let wd = qd[0]; let xd = qd[1]; let yd = qd[2]; let zd = qd[3];

            // Calculate angular rates (omega) from q and q_dot
            //    omega_vec = 2 * conjugate(q) * q_dot
            let rollspeed  = 2.0 * (w*xd - x*wd - y*zd + z*yd);
            let pitchspeed = 2.0 * (w*yd - x*zd - y*wd + z*xd);
            let yawspeed   = 2.0 * (w*zd - x*yd - y*xd + z*wd);

            let msg = comm_messages::messages::AttitudeQuaternionMsg {
                time_boot_ms: (imu_packet.header.timestamp / 1000) as u32,  // Use IMU timestamp, not now_us
                q1: w,
                q2: x,
                q3: y,
                q4: z,
                rollspeed: rollspeed,
                pitchspeed: pitchspeed,
                yawspeed: yawspeed,
            };
            self.send_rosflight_attitude_quaternion(board, msg);
        }

        // --- Send Baro Telemetry ---
        // Interval timer commented out to match C's event-driven architecture
        // C sends immediately when new data is available (got.baro flag)
        // if now_us >= self.last_baro_send_us + BARO_INTERVAL_US {
            let baro_packet_option: &Option<packets::BaroPacket> =
                processed_sensors.get();

            if let Some(baro_packet) = baro_packet_option {
                let baro_msg = comm_messages::messages::SmallBaroMsg {
                    altitude: 0.0f32,
                    pressure: baro_packet.pressure,
                    temperature: baro_packet.temperature,
                };
                self.send_rosflight_small_baro(board, baro_msg);
            }
            // self.last_baro_send_us = now_us;
        // }

        // --- Send Mag Telemetry ---
        // Interval timer commented out to match C's event-driven architecture
        // if now_us >= self.last_mag_send_us + MAG_INTERVAL_US {
            let mag_packet_option: &Option<packets::MagPacket> = processed_sensors.get();
            if let Some(packet) = *mag_packet_option {
                let msg = comm_messages::messages::SmallMagMsg {
                    xmag: packet.flux[0],
                    ymag: packet.flux[1],
                    zmag: packet.flux[2],
                };
                self.send_rosflight_small_mag(board, msg);
            }
            // self.last_mag_send_us = now_us;
        // }

        // --- Send Sonar Telemetry ---
        // Interval timer commented out to match C's event-driven architecture
        // if now_us >= self.last_sonar_send_us + SONAR_INTERVAL_US {
            let range_packet_option: &Option<packets::RangePacket> = processed_sensors.get();
            if let Some(packet) = *range_packet_option {
                let msg = comm_messages::messages::SmallRangeMsg {
                    type_: comm_messages::enums::RosflightRangeType::RosflightRangeSonar,
                    range: packet.range,
                    // TODO: These values aren't in the generic RangePacket.
                    // You may need to add them or send 0.
                    max_range: 0.0,
                    min_range: 0.0,
                };
                self.send_rosflight_small_range(board, msg);
            }
            // self.last_sonar_send_us = now_us;
        // }

        // --- Send Battery Telemetry ---
        // Interval timer commented out to match C's event-driven architecture
        // if now_us >= self.last_battery_send_us + BATTERY_INTERVAL_US {
            let battery_packet_option: &Option<packets::BatteryPacket> = processed_sensors.get();
            if let Some(packet) = *battery_packet_option {
                let msg = comm_messages::messages::BatteryStatusMsg {
                    battery_voltage: packet.voltage,
                    battery_current: packet.current,
                };
                self.send_rosflight_battery_status(board, msg);
            }
            // self.last_battery_send_us = now_us;
        // }

        // --- Send GNSS Telemetry ---
        // Interval timer commented out to match C's event-driven architecture
        // if now_us >= self.last_gnss_send_us + GNSS_INTERVAL_US {
            let gnss_packet_option: &Option<packets::GNSSPacket> = processed_sensors.get();
            if let Some(packet) = *gnss_packet_option {
                let msg = comm_messages::messages::RosflightGnssMsg {
                    rosflight_timestamp: packet.header.timestamp,
                    seconds: packet.sec as u64,
                    nanos: packet.nano as u32,
                    // TODO: This message is complex. You will need to map
                    // fields from your GNSSPacket to this message.
                    // This is just a placeholder structure.
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
                    // ... fill other fields as available ...
                };
                self.send_rosflight_gnss(board, msg);
            }
            // self.last_gnss_send_us = now_us;
        // }

        // --- Send RC Telemetry ---
        // Interval timer commented out to match C's event-driven architecture
        // if now_us >= self.last_rc_send_us + RC_INTERVAL_US {
            // Get the raw f32 packet
            let rc_packet_option: &Option<packets::RcPacket> = processed_sensors.get();
            if let Some(packet) = *rc_packet_option {

                // Create the destination array for u16 channels
                let mut scaled_channels: [u16; RC_PACKET_CHANNELS] = [0; RC_PACKET_CHANNELS];
                let num_channels_to_scale = (packet.n_chan as usize).min(16); // Param arrays are size 16

                // Scale each channel that we have params for
                for i in 0..num_channels_to_scale {
                    // Scale f32 from [-1, 1] to [1000, 2000] (as f32)
                    let scaled_f32 = packet.chan[i] * 1000.0 + 1000.0;
                    // Cast to u16
                    scaled_channels[i] = scaled_f32 as u16;
                }

                // Copy any remaining channels (16-17) as-is (though they are likely 0)
                if packet.n_chan as usize > 16 {
                    for i in 16..packet.n_chan as usize {
                         if i < RC_PACKET_CHANNELS {
                            // We don't have scaling params, just cast the raw value
                            // This will likely just be 0
                            scaled_channels[i] = packet.chan[i] as u16;
                         }
                    }
                }

                let msg = comm_messages::messages::RcChannelsMsg {
                    time_boot_ms: (packet.header.timestamp / 1000) as u32,
                    chancount: packet.n_chan as u8,
                    channels: scaled_channels, // The new [u16; N] array
                    // TODO: The RcPacket does not contain RSSI.
                    // You may need to add this from another source if available.
                    rssi: 255, // 255 = Invalid
                };
                self.send_rosflight_rc_channels(board, msg);
            }
            // self.last_rc_send_us = now_us;
        // }

        // --- Send Output Raw Telemetry ---
        // Interval timer commented out to match C's event-driven architecture
        // if now_us >= self.last_output_raw_us + OUTPUT_RAW_INTERVAL_US {
            let outputs: &[f64] = actuator_commands.as_ref();
            
            // 1. Create the destination array with the new size
            let mut values: [f32; 14] = [0.0; 14];
            
            // 2. Copy up to 14 channels
            let count = outputs.len().min(14);
            //values[..count].copy_from_slice(&outputs[..count]);
            let count = outputs.len().min(14);
            for i in 0..count {
                values[i] = outputs[i] as f32;
            }

            let msg = comm_messages::messages::RosflightOutputRawMsg {
                stamp: now_us, // 3. Use the u64 timestamp
                values,        // 4. Pass the [f32; 14] array
            };
            self.send_rosflight_output_raw(board, msg);
            // self.last_output_raw_us = now_us;
        // }

    }

    pub fn process_incoming_messages(&mut self, board: &mut B) {
        self.comm_link
            .handle_incoming_messages(board, &mut self.msgs);
    }

    pub fn act_on_messages(&mut self, params_iter: &mut Option<ParamIter>, params: &mut Params, cal_flags: &mut CalibrationFlags, board: &mut B, command_manager: &mut CommandManager) -> Option<ParamId> {

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

        // offboard control message received... pass to the command_manager 
        if let Some(msg) = self.msgs.offboard_control.take() {
            let now_us = board.clock_micros();
            command_manager.set_new_offboard_command(now_us, &msg, &params);
        }

        let msg_opt: Option<ParamSetMsg> = self.msgs.param_set.take();
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

                        // MAVLink spec requires acknowledging the change by sending PARAM_VALUE
                        // Find the ParamDefinition to get the ID and count
                        if let Some(def) = PARAM_DEFINITIONS.iter().find(|d| d.name == param_name_str) {
                            
                            if def.id == ParamId::PARAM_SYSTEM_ID {
                                // ...update our internal sysid
                                if let ParamValue::Int(new_sysid) = msg.param_value {
                                    self.sysid = new_sysid as u8;
                                    // println!("CommManager sysid updated to {}", self.sysid);
                                }
                            }

                            let value_msg = ParamValueMsg {
                                param_id: msg.param_id, // Use the received ID bytes
                                param_value: msg.param_value, // Use the value that was set
                                param_count: PARAMS_COUNT as u16,
                                param_index: def.id as u16,
                            };
                            // Assuming 'self.comm_link' and 'board' are accessible
                            // Need system ID - retrieve from params or store in CommManager
                            self.comm_link.send_named_value(board, self.sysid, value_msg);
                            return Some(def.id)
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

        None
    }

    pub fn send_param_value(&mut self, def: &ParamDefinition, val: ParamValue, board: &mut B) {
        let msg = ParamValueMsg {
            param_id: str_to_fixed_bytes(def.name),
            param_value: val,
            param_count: PARAMS_COUNT as u16,
            param_index: def.id as u16,
        };
        self.comm_link.send_named_value(board, self.sysid, msg);
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

    pub fn send_rosflight_attitude_quaternion(&mut self, board: &mut B, msg: AttitudeQuaternionMsg) {
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

    pub fn send_statustext(&mut self, board: &mut B, severity: Severity, text: &str) {
        let text_bytes = {
            let mut arr = [0u8; 50];
            let len = text.len().min(50);
            arr[..len].copy_from_slice(&text.as_bytes()[..len]);
            arr
        };
        
        self.comm_link.send_statustext(board, self.sysid, StatustextMsg { 
            severity, 
            text: text_bytes 
        });
    }

}
