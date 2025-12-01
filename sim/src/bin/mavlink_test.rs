// /**
// ******************************************************************************
// * File     : mavlink_test.rs
// * Date     : November 14, 2025
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

use cdr::{CdrLe, Infinite};
use serde::{Deserialize, Serialize};
use sim::board;
use zenoh::bytes::ZBytes;

use rustflight_core::{
    board::BoardTrait,
    bodytype::{quadrotor::Quadrotor, BodyType},
    comm_manager::comm_link_trait::{mavlink::MavlinkInterface, CommInterface},
    controller::{Controller, quad_controller::QuadController},
    estimator::{Estimator, quad_estimator::QuadEstimator},
    params2::Params,
    pwm::PwmDriver,
    state_machine::StateManager,
    hlist::{Here, There},
    hlist_type,
    mixer::{Mixer, quad_mixer::{QuadMixer}},
    rustflight::{rustflight_typed::ROSFlight, Configuration},
};
use sim::pwm::SimPwmDriver;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
struct SimpleBoolResponse {
    result: bool,
}

// define the wiring diagram
#[derive(Default)]
pub struct SimQuadConfig;
impl Configuration<board::Board, Quadrotor> for SimQuadConfig {
    type SculptIndices = hlist_type![
        Here, // Imu for Estimator
        Here,  // Mag for Estimator
        There<There<There<There<There<Here>>>>> // RC Index
    ];

    type RcPacketSculptedIndex = There<There<Here>>; // RC Index from Sculpted Set

    // --- IMPLEMENT TELEMETRY INDICES ---
    type ImuPacketIndex       = Here;                                                         // index 0
    type MagPacketIndex       = There<Here>;                                                  // index 1
    type BaroPacketIndex      = There<There<Here>>;                                           // index 2
    type PitotPacketIndex     = There<There<There<Here>>>;                                    // index 3
    type RangePacketIndex     = There<There<There<There<Here>>>>;                             // index 4
    type GNSSPacketIndex      = There<There<There<There<There<Here>>>>>;                      // index 5
    type BatteryPacketIndex   = There<There<There<There<There<There<Here>>>>>>;               // index 6
    type RcPacketIndex        = There<There<There<There<There<There<There<Here>>>>>>>;        // index 7
    type AttitudePacketIndex  = There<There<There<There<There<There<There<There<Here>>>>>>>>; // index 8
}

#[tokio::main]
async fn main() {
    // board implementation
    let board = board::Board::new().await;
    let mut pwm_driver = SimPwmDriver::new(&board.zenoh_session).await;

    // Immediately disable ALL channels
    for channel in 0..pwm_driver.len() {
        pwm_driver.disable(channel);
    }

    let mut params = Params::new();
    
    // initialize the timing of the highest level loop through a tick callback 
    let tick_handler = board
        .zenoh_session
        .declare_subscriber("rust/tick")
        .await
        .unwrap();

    // body type instantiations
    let estimator = QuadEstimator::default();
    let controller = QuadController::default();
    let mixer = QuadMixer::new(&params);

    // zero-sized configuration marker (necessary)
    let config = SimQuadConfig::default();

    // comm_link implementation
    let mavlink = MavlinkInterface::new();

    let state_manager = StateManager::new();

    let mut rosflight = ROSFlight::init(1000, board, params, mavlink, state_manager, estimator, controller, mixer, config, pwm_driver);

    //let mut x: u64 = 0;

    while let Ok(_tick) = tick_handler.recv_async().await {
        //println!("tick {}", x);
        //x += 1;

        rosflight.run();

        let response = SimpleBoolResponse { result: true };
        let zb = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&response, Infinite).unwrap());

    }
}
