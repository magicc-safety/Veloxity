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
    board::BoardTrait, 
    bodytype::BodyType,
    comm_manager::{self, comm_link_trait::CommInterface}, 
    comm_messages::{self, messages::HeartbeatMsg}, 
    command_manager::{CommandManager, ControlType}, 
    controller::Controller, 
    errors, 
    estimator::{self, AttitudeStateTrait, Estimator, quad_estimator::{AttitudeState, QuadEstimator}}, 
    hlist::*, 
    mixer::Mixer, 
    packets, 
    params2::{self, PARAM_DEFINITIONS, ParamId, ParamIter}, 
    pwm::{self, PwmDriver}, 
    rc::Rc, 
    rustflight::Configuration, 
    sensorprocessors::CalibrationFlags, 
    state_machine::{self, ErrorFlag, Event, StateManager}
};

const IMU_TIMEOUT_US: u64 = 100_000; // 100ms

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
    last_imu_seen: u64,
    
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
    BT::RequiredSensors: Plucker<Option<packets::RcPacket>, C::RcPacketSculptedIndex>,
    BT::Estimator: Estimator<
        Inputs = <BT::RequiredSensors as Plucker<Option<packets::RcPacket>, C::RcPacketSculptedIndex>>::Remainder,
    >,
    <BT::Estimator as Estimator>::State: AttitudeStateTrait,
    BT::Controller: Controller<State = <BT::Estimator as Estimator>::State>,
    BT::Mixer: Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,
    <<BT as BodyType>::Mixer as Mixer>::ActuatorCommands: AsRef<[f64]>,

    // This tells Rust that the compiler *can* find a way to `get` these packet
    // types using the indices from the `Configuration`.
    B::ProcessedSensorSet: Clone + Sculptor<BT::RequiredSensors, C::SculptIndices>,
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
        rc_manager.init(&mut board, &params);
        let mut command_manager = CommandManager::new();
        command_manager.init(&params, &mut state_manager);
        let now_us = board.clock_micros();
        let mut comm_manager = comm_manager::CommManager::new(comm_link, now_us);

        Self {
            loop_time_us,
            last_imu_seen: now_us,

            board,
            params,
            params_iter: None,
            comm_manager,
            sensors: B::RawSensorSet::default(),
            processorhlist: B::ProcessorHList::default(),
            estimator,
            controller,
            mixer,
            rc_manager,
            command_manager,
            state_manager,
            cal_flags: CalibrationFlags::empty(),
            pwm_driver,
            _body_type: PhantomData,     // field initialization
            _configuration: PhantomData, // field initialization
        }
    }

    pub fn run(&mut self) -> bool {

        let now_ms = self.board.clock_millis();
        let now_us = self.board.clock_micros();

        // act on any received messages this loop
        self.comm_manager.process_incoming_messages(&mut self.board);
        let changed_param_id = self.comm_manager.act_on_messages(
            &mut self.params_iter, 
            &mut self.params, 
            &mut self.cal_flags, 
            &mut self.board,
            &mut self.command_manager,
        );


        if self.state_manager.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO) 
        {
            // this must mean that rc asked for arm, but calibration hadn't happened and needed to... go ahead and raise the calibration flag
            self.cal_flags.insert(CalibrationFlags::GYRO);
        }

        // Data ingestion: let the board update the sensor data store
        // Data processing: run the map operation across HLists
        // This applies the 'ProcessorHList' to the 'RawSensorSet'
        // which consumes the raw data and produces the clean 'ProcessedSensorSet'
        // TODO pass state machine into here... if there's bad sensor data maybe we need to do something about it...
        self.board.update_sensors(&mut self.sensors);
        let processed_sensors = self.sensors.map(&mut self.processorhlist, &mut self.cal_flags, &mut self.params);

       // also check for imu: if it's been too long, add a flag for imu not responding...
        let imu_packet_option: &Option<packets::ImuPacket> = processed_sensors.get();
        if imu_packet_option.is_some() {
            // We got data! Reset the timer and clear the error.
            self.last_imu_seen = now_us;
            self.state_manager.update(Event::ERROR_CLEARED(ErrorFlag::IMU_NOT_RESPONDING), &self.params);
        } else {
            // No data this cycle. Check if the timer has expired.
            if now_us > self.last_imu_seen + IMU_TIMEOUT_US {
                self.state_manager.update(Event::ERROR_OCCURRED(ErrorFlag::IMU_NOT_RESPONDING), &self.params);
            }
        }

        if self.state_manager.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO)
        {
            // The processor has finished! (It removed the flag)
            // We can now send the event to complete the transition.
            // defmt::info!("New Gyro parameters: {}, {}, {}", 
            //     self.params.get_by_id(ParamId::PARAM_GYRO_X_BIAS),
            //     self.params.get_by_id(ParamId::PARAM_GYRO_Y_BIAS),
            //     self.params.get_by_id(ParamId::PARAM_GYRO_Z_BIAS)
            // );
            self.state_manager.update(Event::CALIBRATION_COMPLETE, &self.params);
        }

        let (required_sensors, _remainder) = processed_sensors.clone().sculpt();
        let (rc_packet_option, estimator_sensors) = required_sensors.pluck();

        // now run the RC unit and the command manager unit
        if let Some(rc_packet) = rc_packet_option {
            // defmt::info!("Received RC in rustflight_typed");
            self.rc_manager.receive(
                &rc_packet, 
                &self.params,
                &mut self.state_manager
            );
        }

        self.rc_manager.run(now_ms, &self.params, &mut self.state_manager);
        self.command_manager.run(
            now_ms,
            &self.params, 
            &mut self.rc_manager, 
            &mut self.state_manager);

        // Update the state manager...
        self.state_manager.run(&self.params);

        // Enable PWM after transition to ARMED, disable after transition to DISARMED
        if self.state_manager.is_armed() && !self.pwm_driver.is_enabled() {
            if let Err(_) = self.pwm_driver.enable_all() {
                //defmt::error!("Critical: Failed to enable PWM driver!");
            }
        } else if !self.state_manager.is_armed() && self.pwm_driver.is_enabled() {
            self.pwm_driver.disable_all();
        }

        // Now run the estimator 
        let state = self.estimator.estimate(&estimator_sensors);

        if state.is_healthy() {
            self.state_manager.update(Event::ERROR_CLEARED(ErrorFlag::UNHEALTHY_ESTIMATOR), &self.params);
        } else {
            self.state_manager.update(Event::ERROR_OCCURRED(ErrorFlag::UNHEALTHY_ESTIMATOR), &self.params);
        }

        // Get the final command from the manager, and translate to what the Controller needs:
        let combined_command = self.command_manager.combined_control();
        let controls = self.controller.control(&state, &mut self.state_manager, combined_command, &self.params);
        let actuator_commands = self.mixer.mix(&controls, &mut self.state_manager);

        // PWM command output
        self.pwm_driver.send_commands(&mut self.board, actuator_commands.as_ref());

        // NON-CRITICAL: Logging
        // We limit to 5 logs per loop to prevent a burst of logs from violating 
        // the loop time budget.
        let mut logs_processed = 0;
        while logs_processed < 5 {
            if let Some(entry) = crate::logger::Logger::pop() {
                self.comm_manager.send_statustext(
                    &mut self.board, 
                    entry.severity, 
                    entry.message.as_str()
                );
                logs_processed += 1;
            } else {
                break; 
            }
        }

        self.comm_manager.send_telemetry_streams::<BT, C, _>(
            &mut self.board,
            now_us,
            &self.state_manager,
            &self.command_manager,
            &self.params,
            &state, // The estimator state we just calculated
            &processed_sensors, // The full set of processed sensors
            &actuator_commands, // The final motor commands
        );

        // (We do this *after* telemetry, so telemetry can log if needed)
        if let Some(param_id) = changed_param_id {
            // defmt::info!("Parameter change callback for {:?}", param_id);
            self.rc_manager.param_change_callback(
                param_id, 
                &mut self.board, 
                &self.params, 
                &mut self.comm_manager
            );

            match param_id {
                ParamId::PARAM_FAILSAFE_THROTTLE | ParamId::PARAM_FIXED_WING => {
                    self.command_manager.update_failsafe_config(
                        &self.params, 
                        &mut self.state_manager
                    );
                }
                _ => {
                    // This param change doesn't affect the command manager
                }
            }
            
            // TODO: Add callbacks for other modules here if needed
            // self.controller.param_change_callback(param_id, &self.params);
            // self.estimator.param_change_callback(param_id, &self.params);
        }

        // let the state_manager process it's errors
        self.state_manager.run(&self.params);


        true
    }
}
