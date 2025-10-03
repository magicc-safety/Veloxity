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

use crate::{
    board::BoardTrait,
    bodytype::BodyType,
    comm_manager::{self, comm_link_trait::CommInterface},
    comm_messages,
    controller::Controller,
    errors,
    estimator::Estimator,
    hlist::*,
    mixer::Mixer,
    params2::{self, ParamIter, PARAM_DEFINITIONS},
    rustflight::Configuration,
    sensorprocessors::CalibrationFlags,
    state_machine::{Event, StateManager},
};

pub struct ROSFlight<B, BT, C, CI>
where
    B: BoardTrait,
    BT: BodyType,
    C: Configuration<B, BT>, // The new "glue" constraint
    CI: CommInterface<B>,
{
    loop_time_us: u32,
    pub board: B,
    params: params2::Params,
    params_iter: Option<ParamIter>,
    comm_manager: comm_manager::CommManager<B, CI>,
    sensors: B::RawSensorSet,
    processorhlist: B::ProcessorHList,
    estimator: BT::Estimator,
    controller: BT::Controller,
    mixer: BT::Mixer,
    state_manager: StateManager,
    cal_flags: CalibrationFlags,

    // necessary to tell the compiler these generics are in use.
    _body_type: PhantomData<BT>,
    _configuration: PhantomData<C>,
}

impl<B, BT, C, CI> ROSFlight<B, BT, C, CI>
where
    B: BoardTrait,
    BT: BodyType,
    CI: CommInterface<B>,
    C: Configuration<B, BT>,
    for<'a> B::RawSensorSet: HMappable<'a, B::ProcessorHList, Output = B::ProcessedSensorSet>,
    B::ProcessedSensorSet: Sculptor<BT::RequiredSensors, C::SculptIndices>,
    BT::Estimator: Estimator<Inputs = BT::RequiredSensors>,
    BT::Controller: Controller<State = <BT::Estimator as Estimator>::State>,
    BT::Mixer: Mixer<ControlOutput = <BT::Controller as Controller>::ControlOutput>,
{
    pub fn init(
        loop_time_us: u32,
        mut board: B,
        mut comm_link: CI,
        mut state_manager: StateManager,
        mut estimator: BT::Estimator,
        mut controller: BT::Controller,
        mut mixer: BT::Mixer,
        _config: C, // zero-cost marker for deduction during "init" creation
    ) -> Self { 

        // Initialize all parameters.
        // send a heartbeat to initialize rosflight_io communicaiton 
        let mut params = params2::Params::new();
        comm_link.send_heartbeat(&mut board, 0, comm_messages::messages::HeartbeatMsg { type_: 0, autopilot: 0, base_mode: 0, custom_mode: 0, system_status: 0, mavlink_version: 0 });
        state_manager.update(Event::INITIALIZED, &params);

        Self {
            loop_time_us,
            board,
            params: params2::Params::new(),
            params_iter: None,
            comm_manager: comm_manager::CommManager::new(comm_link),
            sensors: B::RawSensorSet::default(),
            processorhlist: B::ProcessorHList::default(),
            estimator,
            controller,
            mixer,
            state_manager: StateManager::new(),
            cal_flags: CalibrationFlags::empty(),
            _body_type: PhantomData,     // field initialization
            _configuration: PhantomData, // field initialization
        }
    }

    pub fn run(&mut self) -> bool {
        self.comm_manager.process_incoming_messages(&mut self.board);

        // if we haven't already initialized the parameter iteration process, go ahead and start it... otherwise we'll use the iterator we already have
        if self.comm_manager.msgs.param_request_list.take().is_some() {
            if self.params_iter.is_none() {
                self.params_iter = Some(self.params.iter());
            }
        }

        // Parameter sending sequence        
        if let Some(iterator) = &mut self.params_iter {

            // Safely get the next item. This `if let` replaces your `.unwrap()`.
            if let Some((param_id, param_val)) = iterator.next() {
                let def = &PARAM_DEFINITIONS[param_id as usize];
        
                // You now have everything you need to send the message:
                // def.name    -> The parameter's string name (e.g., "SYS_ID")
                // param_id    -> The enum ID (e.g., ParamId::PARAM_SYSTEM_ID)
                // param_val   -> The current value (e.g., ParamValue::Int(1))

                self.comm_manager.send_param_value(def, param_val, &mut self.board);

            } else {
                // The iterator is finished, so set it back to None.
                // This is crucial for preventing future panics and resetting the state.
                self.params_iter = None;
            }
        }

        // Data ingestion: let the board update the sensor data store
        self.board.update_sensors(&mut self.sensors);

        self.state_manager.run(&self.params);

        // Data processing: run the map operation across HLists
        // This applies the 'ProcessorHList' to the 'RawSensorSet'
        // which consumes the raw data and produces the clean 'ProcessedSensorSet'
        let processed_sensors =
            self.sensors
                .map(self.processorhlist, &mut self.cal_flags, &mut self.params);

        let (required_sensors, _remainder) = processed_sensors.sculpt();

        let state = self.estimator.estimate(&required_sensors);
        let controls = self.controller.control(&state);
        let actuator_commands = self.mixer.mix(&controls);

        true
    }
}
