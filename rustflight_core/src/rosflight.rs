// /*
// ******************************************************************************
// * File     : rosflight.rs
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
use crate::{
    board::BoardIo,
    bodytype::BodyType,
    comm_manager::{self, comm_link_trait::CommInterface},
    companion_system::{
        self, AuxCommandCtx, AuxCommandState, CompanionHeartbeatCtx, CompanionLinkState,
        ExternalAttitudeCtx, ExternalAttitudeState,
    },
    command_manager::CommandManager,
    command_system::{
        self, BoardCommandCtx, CalibrationRequestCtx, ConfigInfoCtx, OffboardControlCtx,
        ParamDefaultsCtx, ResetOriginCtx, VersionRequestCtx,
    },
    controller::{Controller, RcTrimCalibrator},
    events::{CommEventQueues, CommandEventQueues, CompanionEventQueues, ParamEventQueues},
    estimator::{
        self, AttitudeStateTrait, NamedEstimator,
        quad_estimator::{AttitudeState, QuadEstimator},
    },
    log_system::{self, LogDrainCtx},
    mixer::Mixer,
    param_reactions::{self, CommandParamChangedCtx, RcParamChangedCtx},
    param_system::{self, ParamApplyCtx, ParamListCtx, ParamListState, ParamReadCtx},
    params2::{self, PARAM_DEFINITIONS, ParamId},
    ports::{EventDrainPort, EventEmitPort, EventReadPort, ParamsReadPort, ParamsWritePort},
    pwm::{self, PwmDriver},
    rc::Rc,
    sensor_systems::{process_sensor_bus, SensorProcessorSet},
    sensorprocessors::CalibrationFlags,
    sensors::{ProcessedSensors, SensorBus},
    state_machine::{self, ErrorFlag, Event, StateManager},
};
use core::marker::PhantomData;
use micro_algebra::stack::vector::Vector;

const IMU_TIMEOUT_US: u64 = 100_000; // 100ms
const ESTIMATOR_DT: f64 = 1.0 / 400.0; // Assume constant 400Hz for estimator

/// A compatibility marker for the legacy `ROSFlight` constructor.
pub trait Configuration<B: crate::board::BoardIo, BT: crate::bodytype::BodyType> {}

pub struct ROSFlight<B, BT, C, CI, PD>
//pub struct ROSFlight<B, BT, C, CI>
where
    B: BoardIo,
    BT: BodyType,
    C: Configuration<B, BT>, // The new "glue" constraint
    CI: CommInterface<B>,
    PD: PwmDriver,
{
    loop_time_us: u32,
    last_imu_seen: u64,
    last_imu_time: u64, // Track last IMU timestamp to detect new data
    // dt: f64,  // Commented out - now using constant ESTIMATOR_DT instead of calculated dt
    pub board: B,
    params: params2::Params,
    param_list_state: ParamListState,
    param_events: ParamEventQueues,
    comm_events: CommEventQueues,
    command_events: CommandEventQueues,
    companion_events: CompanionEventQueues,
    companion_link: CompanionLinkState,
    aux_commands: AuxCommandState,
    external_attitude: ExternalAttitudeState,
    comm_manager: comm_manager::CommManager<B, CI>,
    raw_sensors: SensorBus,
    processed_sensors: ProcessedSensors,
    sensor_processors: SensorProcessorSet,
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
    B: BoardIo,
    BT: BodyType,
    CI: CommInterface<B>,
    C: Configuration<B, BT>,
    PD: PwmDriver,
    BT::Estimator: NamedEstimator,
    <BT::Estimator as NamedEstimator>::State: AttitudeStateTrait,
    BT::Controller: Controller<State = <BT::Estimator as NamedEstimator>::State> + RcTrimCalibrator,
    BT::Mixer: Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,
    <<BT as BodyType>::Mixer as Mixer>::ActuatorCommands: AsRef<[f64]>,
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
            last_imu_time: 0, // Initialize to 0
            // dt: 0.0,  // Commented out - now using constant ESTIMATOR_DT
            board,
            params,
            param_list_state: ParamListState::default(),
            param_events: ParamEventQueues::default(),
            comm_events: CommEventQueues::default(),
            command_events: CommandEventQueues::default(),
            companion_events: CompanionEventQueues::default(),
            companion_link: CompanionLinkState::default(),
            aux_commands: AuxCommandState::default(),
            external_attitude: ExternalAttitudeState::default(),
            comm_manager,
            raw_sensors: SensorBus::default(),
            processed_sensors: ProcessedSensors::default(),
            sensor_processors: SensorProcessorSet::default(),
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
        ///*
        self.board.set_test_pin_2(true);

        let now_ms = self.board.clock_millis();
        let now_us = self.board.clock_micros();

        // act on any received messages this loop
        self.comm_manager.process_incoming_messages(&mut self.board);
        self.comm_manager.act_on_messages(
            &mut self.param_events,
            &mut self.comm_events,
            &mut self.command_events,
            &mut self.companion_events,
            &mut self.board,
        );

        companion_system::apply_companion_heartbeats(CompanionHeartbeatCtx {
            requests: EventDrainPort::new(&mut self.companion_events.heartbeats),
            state: &mut self.companion_link,
        });
        companion_system::apply_aux_commands(AuxCommandCtx {
            requests: EventDrainPort::new(&mut self.companion_events.aux_commands),
            state: &mut self.aux_commands,
        });
        companion_system::apply_external_attitudes(ExternalAttitudeCtx {
            requests: EventDrainPort::new(&mut self.companion_events.external_attitudes),
            state: &mut self.external_attitude,
        });

        let started_calibration = command_system::apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut self.command_events.calibration_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state_manager,
            flags: &mut self.cal_flags,
        });
        self.comm_manager
            .set_pending_calibration_ack(started_calibration);
        command_system::apply_offboard_control_requests(OffboardControlCtx {
            requests: EventDrainPort::new(&mut self.command_events.offboard_control_requests),
            command: &mut self.command_manager,
            params: &self.params,
        });
        command_system::apply_param_defaults_requests(ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut self.command_events.param_defaults_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state_manager,
            params: &mut self.params,
        });

        command_system::apply_rc_trim_calibration_requests(command_system::RcTrimCalibrationCtx {
            requests: EventDrainPort::new(&mut self.command_events.rc_trim_calibration_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state_manager,
            command: &self.command_manager,
            controller: &mut self.controller,
            params: &mut self.params,
        });

        command_system::apply_board_command_requests(BoardCommandCtx {
            requests: EventDrainPort::new(&mut self.command_events.board_command_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state_manager,
            board: &mut self.board,
            params: &mut self.params,
        });

        command_system::apply_version_requests(VersionRequestCtx {
            requests: EventDrainPort::new(&mut self.command_events.version_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state_manager,
        });

        command_system::apply_reset_origin_requests(ResetOriginCtx {
            requests: EventDrainPort::new(&mut self.command_events.reset_origin_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        command_system::apply_config_info_requests(ConfigInfoCtx {
            requests: EventDrainPort::new(&mut self.command_events.config_info_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_system::service_param_read_requests(ParamReadCtx {
            params: ParamsReadPort::new(&self.params),
            requests: EventDrainPort::new(&mut self.param_events.read_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_system::service_param_list_requests(ParamListCtx {
            params: ParamsReadPort::new(&self.params),
            state: &mut self.param_list_state,
            requests: EventDrainPort::new(&mut self.param_events.list_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_system::apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut self.params),
            requests: EventDrainPort::new(&mut self.param_events.set_requests),
            changes: EventEmitPort::new(&mut self.param_events.changes),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_reactions::rc_on_param_changed(RcParamChangedCtx {
            rc: &mut self.rc_manager,
            params: ParamsReadPort::new(&self.params),
            changes: EventReadPort::new(&self.param_events.changes),
        });

        param_reactions::command_on_param_changed(CommandParamChangedCtx {
            command: &mut self.command_manager,
            state: &mut self.state_manager,
            params: ParamsReadPort::new(&self.params),
            changes: EventReadPort::new(&self.param_events.changes),
        });

        self.param_events.changes.clear();

        if self.state_manager.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO) {
            // this must mean that rc asked for arm, but calibration hadn't happened and needed to... go ahead and raise the calibration flag
            self.cal_flags.insert(CalibrationFlags::GYRO);
        }
        //*/
        self.board.update_sensor_bus(&mut self.raw_sensors);
        process_sensor_bus(
            &mut self.raw_sensors,
            &mut self.processed_sensors,
            &mut self.sensor_processors,
            &mut self.cal_flags,
            &mut self.params,
        );

        // also check for imu: if it's been too long, add a flag for imu not responding...
        let imu_packet_option = &self.processed_sensors.imu;
        if imu_packet_option.is_some() {
            //self.board.set_test_pin_1(true);
            // We got data! Reset the timer and clear the error.
            self.last_imu_seen = now_us;
            self.state_manager.update(
                Event::ERROR_CLEARED(ErrorFlag::IMU_NOT_RESPONDING),
                &self.params,
            );
            //self.board.set_test_pin_1(false);
        } else {
            // No data this cycle. Check if the timer has expired.
            if now_us > self.last_imu_seen + IMU_TIMEOUT_US {
                self.state_manager.update(
                    Event::ERROR_OCCURRED(ErrorFlag::IMU_NOT_RESPONDING),
                    &self.params,
                );
            }
        }

        if self.state_manager.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO) {
            // The processor has finished! (It removed the flag)
            // We can now send the event to complete the transition.
            // defmt::info!("New Gyro parameters: {}, {}, {}",
            //     self.params.get_by_id(ParamId::PARAM_GYRO_X_BIAS),
            //     self.params.get_by_id(ParamId::PARAM_GYRO_Y_BIAS),
            //     self.params.get_by_id(ParamId::PARAM_GYRO_Z_BIAS)
            // );
            self.state_manager
                .update(Event::CALIBRATION_COMPLETE, &self.params);
        }

        self.comm_manager
            .queue_completed_calibration_ack(&mut self.comm_events, self.cal_flags);
        self.comm_manager
            .send_comm_responses(&mut self.board, &mut self.comm_events);

        // now run the RC unit and the command manager unit
        if let Some(rc_packet) = self.processed_sensors.rc {
            // defmt::info!("Received RC in rustflight_typed");
            self.rc_manager
                .receive(&rc_packet, &self.params, &mut self.state_manager);
        }

        self.rc_manager
            .run(now_ms, &self.params, &mut self.state_manager);
        self.command_manager.run(
            now_ms,
            &self.params,
            &mut self.rc_manager,
            &mut self.state_manager,
        );

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

        // Check if we have NEW IMU data
        // NOTE: We assume constant 400Hz IMU rate (dt = 1/400 = 0.0025s) for both estimator and controller.
        // The C implementation calculates dt from timestamps, but doesn't handle dropped packets specially
        // (it just uses 2× or 3× dt if packets are dropped). Using constant dt is safer because:
        // - Dropped packet = missed update (same net effect as C)
        // - Avoids potential instability from variable integration timesteps
        let got_new_imu = if let Some(imu_packet) = imu_packet_option {
            let current_time = imu_packet.header.timestamp;
            let is_new = current_time != self.last_imu_time;
            if is_new {
                // Original dt calculation (commented out - now using constant ESTIMATOR_DT)
                // if self.last_imu_time > 0 {
                //     self.dt = (current_time - self.last_imu_time) as f64 * 1e-6;
                // } else {
                //     self.dt = 0.0;  // First IMU packet, no dt yet
                // }
                self.last_imu_time = current_time;
            }
            is_new
        } else {
            false
        };

        // Only run control pipeline when we have NEW IMU data (matches C implementation)
        if got_new_imu {
            let start_time_us = self.board.clock_micros();

            // Run the estimator with constant dt (assume 400Hz)
            let state = self
                .estimator
                .estimate_named(&self.processed_sensors, &self.params, ESTIMATOR_DT);

            // Health check
            if state.is_healthy() {
                self.state_manager.update(
                    Event::ERROR_CLEARED(ErrorFlag::UNHEALTHY_ESTIMATOR),
                    &self.params,
                );
            } else {
                self.state_manager.update(
                    Event::ERROR_OCCURRED(ErrorFlag::UNHEALTHY_ESTIMATOR),
                    &self.params,
                );
            }

            // Get the final command from the manager
            let combined_command = self.command_manager.combined_control();

            // Run controller with same constant dt (assume 400Hz)
            let controls = self.controller.control(
                &state,
                &mut self.state_manager,
                combined_command,
                &self.params,
                ESTIMATOR_DT,
            );

            // Run mixer
            let actuator_commands = self.mixer.mix(&controls, &mut self.state_manager);

            // PWM command output
            self.pwm_driver
                .send_commands(&mut self.board, actuator_commands.as_ref());

            // Measure loop time for this control cycle
            self.loop_time_us = (self.board.clock_micros() - start_time_us) as u32;

            log_system::drain_logs_to_comm_responses(LogDrainCtx {
                responses: EventEmitPort::new(&mut self.comm_events.responses),
            });
            self.comm_manager
                .send_comm_responses(&mut self.board, &mut self.comm_events);

            // Send telemetry with the new state
            self.comm_manager.send_named_telemetry_streams(
                &mut self.board,
                now_us,
                &self.state_manager,
                &self.command_manager,
                &state,             // The estimator state we just calculated
                &self.processed_sensors, // The full set of processed sensors
                &actuator_commands, // The final motor commands
            );
        }
        // End of got_new_imu conditional block

        // let the state_manager process it's errors
        self.state_manager.run(&self.params);

        self.board.set_test_pin_2(false);

        //*/
        true
    }
}
