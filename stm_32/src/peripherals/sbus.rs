// /**
// ******************************************************************************
// * File     : sbus.rs
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
// THIS CODE HAS BEEN MADE SAFE BUT SAFETY HAS NOT BEEN TESTED
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Instant;

use rustflight_core::errors;
use rustflight_core::packets;

pub static RC_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::RcPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::RcPacket, errors::SensorError>>::new();

pub const SBUS_11_BIT_CHANNELS: usize = 16;
pub const SBUS_BINARY_CHANNELS: usize = 2; // does not include status information
pub const SBUS_CHANNELS: usize = SBUS_11_BIT_CHANNELS + SBUS_BINARY_CHANNELS;

pub struct SbusRC {
    pub uart: UartRx<'static, Async>,
}

// Brute force extract of 11 bits
pub fn extract_chan(bytes: &[u8], bit_offset: usize) -> f32 {
    // Calculate the start byte and the bit position within that byte
    let byte_offset = bit_offset / 8;
    let bit_start = bit_offset % 8;

    let mut value: u16 = 0;
    for i in 0..11 {
        let byte_index = byte_offset + (bit_start + i) / 8;
        let bit_index = (bit_start + i) % 8;
        let bit = (bytes[byte_index] >> bit_index) & 1;
        value |= (bit as u16) << i;
    }

    value as f32
}

impl SbusRC {
    pub async fn run(&mut self) {
        let mut buffer = [0u8; 25];

        let mut chan = [0f32; SBUS_CHANNELS];
        let mut timeout = Instant::now() + Duration::from_secs(1);
        let mut rc_chan = [0.0f32; packets::RC_PACKET_CHANNELS];
        loop {
            // Read a packet
            let result = self.uart.read_until_idle(&mut buffer).await;
            if let Ok(size) = result {
                timeout = Instant::now() + Duration::from_secs(1);
                if (buffer[0] == 0x0F)
                    && ((buffer[24] == 0x00)
                        || (buffer[24] == 0x04)
                        || (buffer[24] == 0x14)
                        || (buffer[24] == 0x24)
                        || (buffer[24] == 0x34))
                {
                    let dig = buffer[23] as u16;

                    // get 16 servo (11-bit) channels
                    for i in 0..SBUS_11_BIT_CHANNELS {
                        chan[i] = extract_chan(&buffer, 8 + i * 11);
                    }
                    // get the two binary channels
                    chan[0 + SBUS_11_BIT_CHANNELS] = (((dig) & 0x01) as f32) * 1638.0 + 172.0; // rosflight weird scaling
                    chan[1 + SBUS_11_BIT_CHANNELS] = (((dig >> 1) & 0x01) as f32) * 1638.0 + 172.0; // rosflight weird scaling

                    let header = packets::RosflightPacketHeader {
                        timestamp: Instant::now().as_micros(),
                        status: dig,
                    };

                    rc_chan = [0.0f32; packets::RC_PACKET_CHANNELS];

                    let mut len = SBUS_CHANNELS;

                    if SBUS_CHANNELS > packets::RC_PACKET_CHANNELS {
                        len = packets::RC_PACKET_CHANNELS;
                    }

                    for i in 0..len {
                        rc_chan[i] = chan[i];
                    }

                    let mut rc_packet = packets::RcPacket {
                        header,
                        n_chan: 24,
                        chan: rc_chan,
                        lol: (dig & 0x0C) != 0, // either bit 2 or 3 will signal a loss of link.
                    };
                    RC_SIGNAL.signal(Ok(rc_packet));
                }
            }

            if Instant::now() > timeout {
                timeout = Instant::now() + Duration::from_secs(1);
                let dig = 0x1C; // set bitfield for timeout
                let header = packets::RosflightPacketHeader {
                    timestamp: Instant::now().as_micros(),
                    status: dig,
                };
                let mut rc_packet = packets::RcPacket {
                    header,
                    n_chan: 24,
                    chan: rc_chan,          // last known good values
                    lol: (dig & 0x1C) != 0, // signal a loss of link bits
                };
                RC_SIGNAL.signal(Ok(rc_packet));
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut sbus: SbusRC) {
    sbus.run().await;
}
