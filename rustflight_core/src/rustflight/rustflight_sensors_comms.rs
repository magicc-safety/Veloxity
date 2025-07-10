// /**
// ******************************************************************************
// * File     : rustflight_heartbeat.rs
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
use crate::{
    board::BoardTrait, comm_manager, comm_manager::comm_link_trait::CommInterface, errors, packets,
    params, sensors,
};

pub struct ROSFlight<B, T>
where
    B: BoardTrait,
    T: CommInterface<B>,
{
    loop_time_us: u32,
    pub board: B, // <-- made public on purpose: so that the tests we write aren't subject to the loop. we need to pull both board and comm_link out...
    params: params::Params,
    comm_manager: comm_manager::CommManager<B, T>,
    sensors: sensors::Sensors,
}

impl<B, T> ROSFlight<B, T>
where
    B: BoardTrait,
    T: CommInterface<B>,
{
    pub fn init(_loop_time_us: u32, board: B, comm_link: T) -> Self {
        Self {
            loop_time_us: _loop_time_us,
            board,
            params: params::Params::new(),
            comm_manager: comm_manager::CommManager::new(comm_link),
            sensors: sensors::Sensors::new(),
        }
    }

    pub fn run(&mut self) -> bool {
        self.sensors.run(&mut self.board);
        self.comm_manager.process_incoming_messages(&mut self.board);
        self.comm_manager.send_heartbeat(&mut self.board);
        return true;
    }
}
