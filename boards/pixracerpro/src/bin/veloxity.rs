// ******************************************************************************
// * File     : boards/pixracerpro/src/bin/veloxity.rs
// * Date     : June 28, 2026
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

#![no_std]
#![no_main]
use cortex_m_rt::entry;
use panic_halt as _;
use pixracerpro::pwm::BoardPwmDriver;
use pixracerpro::*;
use stm_32::*;
use veloxity_core::world::ControlLoopRates;
use veloxity_core::world::RealtimeSchedulerStep;
use veloxity_core::{
    board::BoardIo,
    params::Params,
    state_machine::StateManager,
    vehicle::quadrotor,
    world::{RealtimeServicePolicy, World},
};
use veloxity_mavlink::MavlinkInterface;

type PixracerReal = f64;
const PIXRACER_CONTROL_LOOP_HZ: u16 = 400;

type PixracerWorld<'a> = World<
    board::Board,
    quadrotor::Estimator<PixracerReal>,
    quadrotor::Controller<PixracerReal>,
    quadrotor::Mixer<PixracerReal>,
    MavlinkInterface,
    BoardPwmDriver<'a>,
    PixracerReal,
>;

fn init_world<'a>(
    board: board::Board,
    params: Params,
    pwm_driver: BoardPwmDriver<'a>,
) -> PixracerWorld<'a> {
    let mixer = quadrotor::mixer(&params);
    PixracerWorld::init(
        board,
        params,
        MavlinkInterface::new(),
        StateManager::new(),
        quadrotor::Estimator::default(),
        quadrotor::Controller::default(),
        mixer,
        pwm_driver,
    )
}

#[entry]
fn main() -> ! {
    // board implementation & servos object
    let (mut board, mut servos) = board::Board::new();
    let mut params = Params::default();
    if !board.read_params(&mut params) {
        veloxity_core::log_warn!("Unable to load parameters; using default values");
        params.set_defaults();
        let _ = board.write_params(&params);
    }

    // PWM Driver from servos object
    let pwm_driver = BoardPwmDriver::new(&mut servos);

    let mut world = init_world(board, params, pwm_driver);
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(PIXRACER_CONTROL_LOOP_HZ));

    loop {
        match world.realtime_scheduler_step() {
            RealtimeSchedulerStep::ImuControl => {
                let _ = world.run_imu_control_tick();
            }
            RealtimeSchedulerStep::ControlUpdate => {
                let _ = world.run_control_update_tick();
            }
            RealtimeSchedulerStep::Service => {
                let _ = world.run_prioritized_service_steps_with_policy(
                    RealtimeServicePolicy::continuous_slack_driven(),
                );
            }
            RealtimeSchedulerStep::Idle => {}
        }
    }
}
