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
//use defmt;
use nucleo::*;
use panic_halt as _;
use rustflight_core::{
    bodytype::quadrotor::Quadrotor,
    comm_manager::comm_link_trait::mavlink::MavlinkInterface,
    controller::Controller,
    controller::quad_controller::QuadController,
    estimator::quad_estimator::QuadEstimator,
    mixer::Mixer,
    mixer::quad_mixer::QuadMixer,
    params2::Params,
    rosflight::ROSFlight,
    state_machine::StateManager,
};
use stm_32::*;

#[entry]
fn main() -> ! {
    // board implementation
    let (board, pwm_driver) = board::Board::new();
    let params = Params::default();

    // body type instantiations
    let estimator = QuadEstimator::default();
    let controller = QuadController::default();
    let mixer = QuadMixer::new(&params);

    // comm_link implementation
    let mavlink = MavlinkInterface::new();

    let state_manager = StateManager::new();

    let mut rosflight = ROSFlight::<_, Quadrotor, _, _>::init(
        1000,
        board,
        params,
        mavlink,
        state_manager,
        estimator,
        controller,
        mixer,
        pwm_driver,
    );

    loop {
        // defmt::debug!("One Loop");
        rosflight.run();
    }
}
