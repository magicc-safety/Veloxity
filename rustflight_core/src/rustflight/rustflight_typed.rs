// /**
// ******************************************************************************
// * File     : rustflight_typed.rs
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
// THIS CODE HAS NOT BEEN MADE SAFE YET
//use crate::mavlink::dialects::rosflight::{self as rosflight_dialect};

use core::marker::PhantomData;
use micro_algebra::stack::vector::Vector;
use crate::{
    board::BoardTrait, bodytype::BodyType, comm_manager::{self, comm_link_trait::CommInterface}, comm_messages::{self, messages::HeartbeatMsg}, command_manager::{CommandManager, ControlType}, controller::Controller, errors, estimator::{self, Estimator, quad_estimator::{AttitudeState, QuadEstimator}}, hlist::*, mixer::Mixer, packets, params2::{self, PARAM_DEFINITIONS, ParamIter}, pwm::{self, PwmDriver}, rc::Rc, rustflight::Configuration, sensorprocessors::CalibrationFlags, state_machine::{Event, StateManager, ErrorFlag}
};

const HEARTBEAT_INTERVAL_US: u64 = 1_000_000; // 1 second = 1,000,000 microseconds
const STATUS_INTERVAL_US: u64 = 500_000;    // 2 Hz
const ATTITUDE_INTERVAL_US: u64 = 10_000;   // 100 Hz
const IMU_INTERVAL_US: u64 = 2500;          // 400 Hz
const BARO_INTERVAL_US: u64 = 20_000;         // 50 Hz

pub struct ROSFlight<B, BT, C, CI, PD>
//pub struct ROSFlight<B, BT, C, CI>
where
    B: BoardTrait,
    BT: BodyType,
    C: Configuration<B, BT>, // The new "glue" constraint
    CI: CommInterface<B>,
    PD: PwmDriver,
{
    loop_time_us: u32,

    last_heartbeat_us: u64,
    last_status_send_us: u64,
    last_imu_send_us: u64,
    last_attitude_send_us: u64,
    last_baro_send_us: u64,

    pub board: B,
    params: params2::Params,
    params_iter: Option<ParamIter>,
    comm_manager: comm_manager::CommManager<B, CI>,
    sensors: B::RawSensorSet,
    processorhlist: B::ProcessorHList,
    estimator: BT::Estimator,
    controller: BT::Controller,
    mixer: BT::Mixer,
    rc_manager: Rc,
    cal_flags: CalibrationFlags,
    command_manager: CommandManager,
    state_manager: StateManager,
    pwm_driver: PD,

    // necessary to tell the compiler these generics are in use.
    _body_type: PhantomData<BT>,
    _configuration: PhantomData<C>,
}

impl<B, BT, C, CI, PD> ROSFlight<B, BT, C, CI, PD>
//impl<B, BT, C, CI> ROSFlight<B, BT, C, CI>
where
    B: BoardTrait,
    BT: BodyType,
    CI: CommInterface<B>,
    C: Configuration<B, BT>,
    PD: PwmDriver,
    for<'a> B::RawSensorSet: HMappable<'a, B::ProcessorHList, Output = B::ProcessedSensorSet>,
    B::ProcessedSensorSet: Sculptor<BT::RequiredSensors, C::SculptIndices>,
    BT::RequiredSensors: Plucker<Option<packets::RcPacket>, C::RcPacketIndex>,
    BT::Estimator: Estimator<
        Inputs = <BT::RequiredSensors as Plucker<Option<packets::RcPacket>, C::RcPacketIndex>>::Remainder,
    >,
    BT::Controller: Controller<State = <BT::Estimator as Estimator>::State>,
    BT::Mixer: Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,

    // This tells Rust that the compiler *can* find a way to `get` these packet
    // types using the indices from the `Configuration`.
    B::ProcessedSensorSet: HListGet<Option<packets::ImuPacket>, C::ImuPacketIndex> +
                           HListGet<Option<packets::MagPacket>, C::MagPacketIndex> +
                           HListGet<Option<packets::BaroPacket>, C::BaroPacketIndex> +
                           HListGet<Option<packets::PitotPacket>, C::PitotPacketIndex> +
                           HListGet<Option<packets::RangePacket>, C::RangePacketIndex> +
                           HListGet<Option<packets::GNSSPacket>, C::GNSSPacketIndex> +
                           HListGet<Option<packets::BatteryPacket>, C::BatteryPacketIndex> +
                           HListGet<Option<packets::AttitudePacket>, C::AttitudePacketIndex>
{
    pub fn init(
        loop_time_us: u32,
        mut board: B,
        mut params: params2::Params,
        mut comm_link: CI,
        mut state_manager: StateManager,
        mut estimator: BT::Estimator,
        mut controller: BT::Controller,
        mut mixer: BT::Mixer,
        _config: C, // zero-cost marker for deduction during "init" creation
        mut pwm_driver: PD,
    ) -> Self { 

        state_manager.update(Event::INITIALIZED, &params);
        let mut rc_manager = Rc::new();
        rc_manager.init(&params);
        let mut command_manager = CommandManager::new();

        let now_us = board.clock_micros();

        Self {
            loop_time_us,

            last_heartbeat_us: now_us,
            last_status_send_us: now_us,
            last_imu_send_us: now_us,
            last_attitude_send_us: now_us,
            last_baro_send_us: now_us,

            board,
            params,
            params_iter: None,
            comm_manager: comm_manager::CommManager::new(comm_link),
            sensors: B::RawSensorSet::default(),
            processorhlist: B::ProcessorHList::default(),
            estimator,
            controller,
            mixer,
            rc_manager,
            command_manager,
            state_manager: StateManager::new(),
            cal_flags: CalibrationFlags::empty(),
            pwm_driver,
            _body_type: PhantomData,     // field initialization
            _configuration: PhantomData, // field initialization
        }
    }

    pub fn run(&mut self) -> bool {

        let now_ms = self.board.clock_millis();
        let now_us = self.board.clock_micros();

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
            self.comm_manager.send_heartbeat(&mut self.board, hb);
            self.last_heartbeat_us = now_us;
        }

        // act on any received messages this loop
        self.comm_manager.process_incoming_messages(&mut self.board);
        self.comm_manager.act_on_messages(&mut self.params_iter, &mut self.params, &mut self.cal_flags, &mut self.board);

        // Handle Status Message
        if now_us >= self.last_status_send_us + STATUS_INTERVAL_US {
            let status_msg = comm_messages::messages::RosflightStatusMsg {
                armed: self.state_manager.is_armed() as u8,
                failsafe: self.state_manager.is_in_failsafe() as u8,
                rc_override: 0, // Placeholder: self.command_manager.is_rc_override() as u8,
                offboard: 0, // Placeholder: self.command_manager.is_offboard() as u8,
                error_code: self.state_manager.get_errors(),
                control_mode: self.command_manager.get_control_mode().into(),
                num_errors: self.state_manager.get_errors().bits().count_ones() as i16,
                loop_time_us: 0, // Placeholder
            };
            self.comm_manager.send_status(&mut self.board, status_msg);
            self.last_status_send_us = now_us;
        }

        // Data ingestion: let the board update the sensor data store
        // Data processing: run the map operation across HLists
        // This applies the 'ProcessorHList' to the 'RawSensorSet'
        // which consumes the raw data and produces the clean 'ProcessedSensorSet'
        // TODO pass state machine into here... if there's bad sensor data maybe we need to do something about it...
        self.board.update_sensors(&mut self.sensors);
        let processed_sensors = self.sensors.map(self.processorhlist, &mut self.cal_flags, &mut self.params);

        let (required_sensors, _remainder) = processed_sensors.sculpt();
        let (rc_packet_option, estimator_sensors) = required_sensors.pluck();
        
        // now run the RC unit and the command manager unit
        if let Some(rc_packet) = rc_packet_option {
            self.rc_manager.receive(&rc_packet, &mut self.state_manager, &self.params);
        }
        self.rc_manager.run(now_ms, &self.params, &mut self.state_manager);
        self.command_manager.run(now_ms, &self.comm_manager, &self.params, &mut self.rc_manager, &self.state_manager);

        // Update the state manager...
        self.state_manager.run(&self.params);

        // Now run the estimator 
        let state= self.estimator.estimate(&estimator_sensors);

        // --- Send Attitude Telemetry (e.g., 100 Hz) ---
        // This uses the *output* of the estimator
        // if now_us >= self.last_attitude_send_us + ATTITUDE_INTERVAL_US {
        //     // TODO: Update `AttitudeState` or `QuadEstimator` to also return angular rates
        //     // The logic to convert q_dot to rates is complex.
        //     let att_msg = comm_messages::messages::AttitudeQuaternionMsg {
        //         time_boot_ms: (now_us / 1000) as u32,
        //         q1: state.q_hat[0] as f32, // w
        //         q2: state.q_hat[1] as f32, // x
        //         q3: state.q_hat[2] as f32, // y
        //         q4: state.q_hat[3] as f32, // z
        //         rollspeed: 0.0,  // Placeholder - state.q_dot is NOT angular rate
        //         pitchspeed: 0.0, // Placeholder
        //         yawspeed: 0.0,   // Placeholder
        //     };
        //     self.comm_manager.comm_link.send_attitude(&mut self.board, sysid, att_msg);
        //     self.last_attitude_send_us = now_us;
        // }

        // Get the final command from the manager, and translate to what the Controller needs:
        let combined_command = self.command_manager.combined_control();
        let controls = self.controller.control(&state, &*combined_command);
        let actuator_commands = self.mixer.mix(&controls);

        // // PWM command output
        self.pwm_driver.send_commands(&mut self.board, actuator_commands.as_ref());

        // let the state_manager process it's errors
        self.state_manager.run(&self.params);

        true
    }
}