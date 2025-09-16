// /**
// ******************************************************************************
// * File     : comm_manager.rs
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
// * list of conditions and the following disclaimer. *
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
pub mod comm_link_trait;
pub mod mavlink_parser;

use crate::board;
use crate::comm_messages;
use core::marker::PhantomData;

pub struct CommManager<B, T>
where
    B: board::BoardTrait,
    T: comm_link_trait::CommInterface<B>,
{
    sysid: u8,
    comm_link: T,
    pub msgs: comm_messages::Messages,
    _board_marker: PhantomData<B>,
}

impl<B, T> CommManager<B, T>
where
    B: board::BoardTrait,
    T: comm_link_trait::CommInterface<B>,
{
    pub fn new(comm_link: T) -> Self {
        CommManager {
            sysid: 0,
            comm_link,
            msgs: comm_messages::Messages::default(),
            _board_marker: PhantomData,
        }
    }

    pub fn process_incoming_messages(&mut self, board: &mut B) {
        self.comm_link
            .handle_incoming_messages(board, &mut self.msgs);
    }

    pub fn send_heartbeat(&mut self, board: &mut B) {
        self.comm_link.send_heartbeat(board, self.sysid, false);
    }
}
