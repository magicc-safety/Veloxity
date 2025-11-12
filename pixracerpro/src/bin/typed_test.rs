#![no_std]
#![no_main]
// /**
// ******************************************************************************
// * File     : estimator_test.rs
// * Date     : October 8, 2025
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
use cortex_m;
use cortex_m_rt::entry;
// use defmt;
use pixracerpro::pwm::BoardPwmDriver;
use pixracerpro::*;
use rustflight_core::{
    board::BoardTrait,
    board::dummy::DummyBoard,
    bodytype::BodyType,
    bodytype::quadrotor::Quadrotor,
    comm_manager::comm_link_trait::mavlink::MavlinkInterface,
    controller::Controller,
    estimator::Estimator,
    hlist::{Here, There},
    hlist_type,
    mixer::Mixer,
    params2::Params,
    pwm,
    rustflight::Configuration,
    rustflight::rustflight_typed::ROSFlight,
    state_machine::StateManager,
};
use stm_32::{peripherals::pwm::PixRacerProServoMonstrosity, *};

// Tiny aliases for readability
pub type I0 = Here;
pub type I1 = There<I0>;
pub type I2 = There<I1>;
pub type I3 = There<I2>;
pub type I4 = There<I3>;
pub type I5 = There<I4>;
pub type I6 = There<I5>;
pub type I7 = There<I6>;
pub type I8 = There<I7>;

// define the wiring diagram
#[derive(Default)]
pub struct PixRacerProQuadConfig;
impl Configuration<board::Board, Quadrotor> for PixRacerProQuadConfig {
    // IMU, Mag, RC
    type SculptIndices = hlist_type![I0, I0, I5];

    // RC is at There<There<Here>> in the Quadrotor Bodytype
    type RcPacketSculptedIndex = I2;

    type ImuPacketIndex = I0;
    type MagPacketIndex = I1;
    type BaroPacketIndex = I2;
    type PitotPacketIndex = I3;
    type RangePacketIndex = I4;
    type GNSSPacketIndex = I5;
    type BatteryPacketIndex = I6;
    type RcPacketIndex = I7;
    type AttitudePacketIndex = I8;
}

#[entry]
fn main() -> ! {
    // board implementation
    let (mut board, mut servos) = board::Board::new();

    // body type instantiations
    let estimator =
        <rustflight_core::bodytype::quadrotor::Quadrotor as BodyType>::Estimator::default();
    let controller =
        <rustflight_core::bodytype::quadrotor::Quadrotor as BodyType>::Controller::default();
    let mixer = <rustflight_core::bodytype::quadrotor::Quadrotor as BodyType>::Mixer::new(
        &Params::default(),
    );

    // zero-sized configuration marker (necessary)
    let config = PixRacerProQuadConfig::default();

    // comm_link implementation
    let mavlink = MavlinkInterface::new();

    // state_manager
    let state_manager = StateManager::new();

    // PWM Driver
    let pwm_driver = BoardPwmDriver::new(&mut servos);

    let mut rosflight = ROSFlight::init(
        1000,
        board,
        Params::default(),
        mavlink,
        state_manager,
        estimator,
        controller,
        mixer,
        config,
        pwm_driver,
    );

    loop {
        // defmt::info!("Hello World!");
        rosflight.run();
    }
}
