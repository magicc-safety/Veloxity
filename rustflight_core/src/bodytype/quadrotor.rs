// /**
// ******************************************************************************
// * File     : dummy.rs
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
use crate::bodytype::BodyType;
use crate::controller::Controller;
use crate::estimator::Estimator;
use crate::hlist_type;
use crate::mixer::Mixer;
use crate::packets;
use crate::packets::*;

pub struct Quadrotor;
pub struct AttitudeState;
pub struct MixerInput;

impl BodyType for Quadrotor {
    // shopping list of required sensors...
    type RequiredSensors = hlist_type![
        Option<packets::ImuPacket>,
        Option<packets::MagPacket>,
        Option<packets::BaroPacket>,
        Option<packets::GNSSPacket>
    ];

    type Estimator = QuadEstimator;
    type Controller = QuadController;
    type Mixer = QuadMixer;
}

#[derive(Default)]
pub struct QuadEstimator;
impl Estimator for QuadEstimator {
    type Inputs = hlist_type![
        Option<packets::ImuPacket>,
        Option<packets::MagPacket>,
        Option<packets::BaroPacket>,
        Option<packets::GNSSPacket>
    ];

    type State = AttitudeState;

    fn estimate(&mut self, inputs: &Self::Inputs) -> Self::State {
        //println!("Estimating!");
        AttitudeState {}
    }
}

#[derive(Default)]
pub struct QuadController;
impl Controller for QuadController {
    type State = AttitudeState;
    type ControlOutput = MixerInput;

    fn control(&mut self, state: &Self::State) -> Self::ControlOutput {
        //println!("Controlling!");
        MixerInput {}
    }
}

#[derive(Default)]
pub struct QuadMixer;
impl Mixer for QuadMixer {
    type ControlOutput = MixerInput;
    type ActuatorCommands = u32;

    fn mix(&mut self, controls: &Self::ControlOutput) -> Self::ActuatorCommands {
        //println!("Mixing!");
        0u32
    }
}
