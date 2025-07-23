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
    controller::Controller,
    errors,
    estimator::Estimator,
    hlist::*,
    mixer::Mixer,
    params,
    rustflight::Configuration,
    sensorprocessors::CalibrationFlags,
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
    params: params::Params,
    comm_manager: comm_manager::CommManager<B, CI>,
    sensors: B::RawSensorSet,
    processorhlist: B::ProcessorHList,
    estimator: BT::Estimator,
    controller: BT::Controller,
    mixer: BT::Mixer,
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
        board: B,
        comm_link: CI,
        estimator: BT::Estimator,
        controller: BT::Controller,
        mixer: BT::Mixer,
        _config: C, // zero-cost marker for deduction during "init" creation
    ) -> Self {
        Self {
            loop_time_us,
            board,
            params: params::Params::new(),
            comm_manager: comm_manager::CommManager::new(comm_link),
            sensors: B::RawSensorSet::default(),
            processorhlist: B::ProcessorHList::default(),
            estimator,
            controller,
            mixer,
            cal_flags: CalibrationFlags::empty(),
            _body_type: PhantomData,     // field initialization
            _configuration: PhantomData, // field initialization
        }
    }

    pub fn run(&mut self) -> bool {
        //self.comm_manager.process_incoming_messages(&mut self.board);
        self.comm_manager.send_heartbeat(&mut self.board);

        // Data ingestion: let the board update the sensor data store
        self.board.update_sensors(&mut self.sensors);

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
