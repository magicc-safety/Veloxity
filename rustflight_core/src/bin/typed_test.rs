// use std::time::Duration;

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
// use rustflight_core::{
//     board::{dummy::DummyBoard, BoardTrait},
//     bodytype::{quadrotor::Quadrotor, BodyType},
//     comm_manager::comm_link_trait::mavlink::MavlinkInterface,
//     controller::{Controller, quad_controller::QuadController},
//     estimator::{Estimator, quad_estimator::QuadEstimator},
//     params2::Params,
//     mixer::{Mixer, quad_mixer::{QuadMixer}},
//     hlist::{Here, There},
//     hlist_type,
//     packets,
//     rc::Rc,
//     rustflight::{rustflight_typed::ROSFlight, Configuration},
//     state_machine::StateManager,
//     pwm::PwmDriver // We need to switch this out for the PwmDriver written for this test.
// };

// // define the wiring diagram
// #[derive(Default)]
// pub struct DummyQuadConfig;
// impl Configuration<DummyBoard, Quadrotor> for DummyQuadConfig {
//     type SculptIndices = hlist_type![
//         Here,
//         Here,
//         There<There<There<There<There<Here>>>>>
//     ];

//     type RcPacketSculptedIndex = There<There<Here>>;

//     type ImuPacketIndex = Here;
//     type MagPacketIndex = There<Here>;
//     type BaroPacketIndex = There<There<Here>>;
//     type PitotPacketIndex = There<There<There<Here>>>;
//     type RangePacketIndex = There<There<There<There<Here>>>>;
//     type GNSSPacketIndex = There<There<There<There<There<Here>>>>>;
//     type BatteryPacketIndex = There<There<There<There<There<There<Here>>>>>>;
//     type RcPacketIndex = There<There<There<There<There<There<There<Here>>>>>>>;
//     type AttitudePacketIndex = There<There<There<There<There<There<There<There<Here>>>>>>>>;
// }

fn main() {}

// fn main() {
//     // board implementation
//     let (board, servos) = DummyBoard::default(); // We need to update the Default to return the (board, servos) tuple
//     let mut params = Params::new();

//     // body type instantiations...
//     let estimator = QuadEstimator::default();
//     let controller = QuadController::default();
//     let mixer = QuadMixer::new(&params);

//     // zero-sized configuration marker (necessary)
//     let config = DummyQuadConfig::default();

//     // comm_link implementation
//     let mavlink = MavlinkInterface::new();

//     let state_manager = StateManager::new();

//     let mut rosflight = ROSFlight::init(1000, board, params, mavlink, state_manager, estimator, controller, mixer, config, PwmDriver::new(&mut servos),); // we need to create a PwmDriver for this test

//     loop {
//         println!("Highest Level Loop");
//         println!("---------------------------------");
//         rosflight.run();
//         println!("---------------------------------");
//         println!("");

//         std::thread::sleep(Duration::from_secs(1));
//     }
// }
