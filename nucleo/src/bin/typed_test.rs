#![no_std]
#![no_main]
// /**
// ******************************************************************************
// * File     : typed_test.rs
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
use cortex_m_rt::entry;
use defmt;
use nucleo::*;
use rustflight_core::{
    board::BoardTrait,
    board::dummy::DummyBoard,
    bodytype::BodyType,
    bodytype::quadrotor::{QuadController, QuadEstimator, QuadMixer, Quadrotor},
    comm_manager::comm_link_trait::mavlink::MavlinkInterface,
    controller::Controller,
    estimator::Estimator,
    hlist::{Here, There},
    hlist_type,
    mixer::Mixer,
    rustflight::Configuration,
    rustflight::rustflight_typed::ROSFlight,
};
use stm_32::*;

// define the wiring diagram
#[derive(Default)]
pub struct NucleoQuadConfig;
impl Configuration<board::Board, Quadrotor> for NucleoQuadConfig {
    // needs IMU, Baro, Mag, GNSS
    type SculptIndices = hlist_type![Here, Here, Here, There<Here>];
}

#[entry]
fn main() -> ! {
    // board implementation
    let mut board = board::Board::new();

    // body type instantiations
    let estimator = QuadEstimator::default();
    let controller = QuadController::default();
    let mixer = QuadMixer::default();

    // zero-sized configuration marker (necessary)
    let config = NucleoQuadConfig::default();

    // comm_link implementation
    let mavlink = MavlinkInterface::new();

    let mut rosflight = ROSFlight::init(1000, board, mavlink, estimator, controller, mixer, config);

    loop {
        defmt::debug!("One Loop");
        rosflight.run();
    }
}
