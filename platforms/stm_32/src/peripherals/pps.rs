// ******************************************************************************
// * File     : platforms/stm_32/src/peripherals/pps.rs
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

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

use veloxity_core::packets;

pub static PPS_SIGNAL: Signal<CriticalSectionRawMutex, packets::PpsPacket> =
    Signal::<CriticalSectionRawMutex, packets::PpsPacket>::new();

fn publish_pps(packet: packets::PpsPacket) {
    #[cfg(feature = "runtime-diagnostics")]
    crate::runtime_diagnostics::record_signal_publish(
        crate::runtime_diagnostics::SensorKind::Pps,
        PPS_SIGNAL.signaled(),
        false,
    );
    PPS_SIGNAL.signal(packet);
}

pub struct PpsSensor {
    pub pps: ExtiInput<'static, Async>,
}

impl PpsSensor {
    pub async fn run(&mut self) {
        loop {
            self.pps.wait_for_rising_edge().await;
            let timestamp = Instant::now();
            let status = 1;
            let header = packets::RosflightPacketHeader {
                timestamp: timestamp.as_micros(),
                status,
            };
            let pps_packet = packets::PpsPacket { header };
            publish_pps(pps_packet);
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut pps: PpsSensor) {
    pps.run().await;
}
