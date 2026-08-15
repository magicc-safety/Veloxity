// ******************************************************************************
// * File     : platforms/stm_32/src/peripherals/sbus.rs
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

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Duration;
use embassy_time::Instant;

use veloxity_core::errors;
use veloxity_core::packets;

const RC_QUEUE_CAPACITY: usize = 8;

pub static RC_CHANNEL: Channel<
    CriticalSectionRawMutex,
    Result<packets::RcPacket, errors::SensorError>,
    RC_QUEUE_CAPACITY,
> = Channel::new();

async fn publish_rc(result: Result<packets::RcPacket, errors::SensorError>) {
    #[cfg(feature = "runtime-diagnostics")]
    let error = result.is_err();
    #[cfg(feature = "runtime-diagnostics")]
    let wait_started = RC_CHANNEL.is_full().then(Instant::now);
    RC_CHANNEL.send(result).await;
    #[cfg(feature = "runtime-diagnostics")]
    crate::runtime_diagnostics::record_rc_queue_publish(
        error,
        wait_started.map(|started| started.elapsed().as_micros().min(u32::MAX as u64) as u32),
        RC_CHANNEL.len(),
    );
}

pub const SBUS_11_BIT_CHANNELS: usize = 16;
pub const SBUS_BINARY_CHANNELS: usize = 2; // does not include status information
pub const SBUS_CHANNELS: usize = SBUS_11_BIT_CHANNELS + SBUS_BINARY_CHANNELS;

static SBUS_READ_OK: AtomicU32 = AtomicU32::new(0);
static SBUS_READ_ERR: AtomicU32 = AtomicU32::new(0);
static SBUS_LAST_READ_SIZE: AtomicU32 = AtomicU32::new(0);
static SBUS_SIZE_25: AtomicU32 = AtomicU32::new(0);
static SBUS_VALID_FRAME: AtomicU32 = AtomicU32::new(0);
static SBUS_BAD_HEADER: AtomicU32 = AtomicU32::new(0);
static SBUS_BAD_FOOTER: AtomicU32 = AtomicU32::new(0);
static SBUS_SIGNAL: AtomicU32 = AtomicU32::new(0);
static SBUS_TIMEOUT: AtomicU32 = AtomicU32::new(0);
static SBUS_LAST_STATUS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Debug, Default)]
pub struct SbusDiagnostics {
    pub read_ok: u32,
    pub read_err: u32,
    pub last_read_size: u32,
    pub size_25: u32,
    pub valid_frame: u32,
    pub bad_header: u32,
    pub bad_footer: u32,
    pub signal: u32,
    pub timeout: u32,
    pub last_status: u32,
}

pub fn diagnostics() -> SbusDiagnostics {
    SbusDiagnostics {
        read_ok: SBUS_READ_OK.load(Ordering::Relaxed),
        read_err: SBUS_READ_ERR.load(Ordering::Relaxed),
        last_read_size: SBUS_LAST_READ_SIZE.load(Ordering::Relaxed),
        size_25: SBUS_SIZE_25.load(Ordering::Relaxed),
        valid_frame: SBUS_VALID_FRAME.load(Ordering::Relaxed),
        bad_header: SBUS_BAD_HEADER.load(Ordering::Relaxed),
        bad_footer: SBUS_BAD_FOOTER.load(Ordering::Relaxed),
        signal: SBUS_SIGNAL.load(Ordering::Relaxed),
        timeout: SBUS_TIMEOUT.load(Ordering::Relaxed),
        last_status: SBUS_LAST_STATUS.load(Ordering::Relaxed),
    }
}

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
                SBUS_READ_OK.fetch_add(1, Ordering::Relaxed);
                SBUS_LAST_READ_SIZE.store(size as u32, Ordering::Relaxed);
                if size == buffer.len() {
                    SBUS_SIZE_25.fetch_add(1, Ordering::Relaxed);
                }
                timeout = Instant::now() + Duration::from_secs(1);
                let valid_header = buffer[0] == 0x0F;
                let valid_footer = (buffer[24] == 0x00)
                    || (buffer[24] == 0x04)
                    || (buffer[24] == 0x14)
                    || (buffer[24] == 0x24)
                    || (buffer[24] == 0x34);
                if valid_header && valid_footer {
                    SBUS_VALID_FRAME.fetch_add(1, Ordering::Relaxed);
                    let dig = buffer[23] as u16;
                    SBUS_LAST_STATUS.store(dig as u32, Ordering::Relaxed);

                    // get 16 servo (11-bit) channels
                    for i in 0..SBUS_11_BIT_CHANNELS {
                        chan[i] = (extract_chan(&buffer, 8 + i * 11) - 172.0) / 1639.0;
                    }
                    // get the two binary channels
                    chan[0 + SBUS_11_BIT_CHANNELS] = ((((dig) & 0x01) as f32) - 172.0) / 1639.0; // rosflight weird scaling
                    chan[1 + SBUS_11_BIT_CHANNELS] =
                        ((((dig >> 1) & 0x01) as f32) - 172.0) / 1639.0; // rosflight weird scaling

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

                    let rc_packet = packets::RcPacket {
                        header,
                        n_chan: 24,
                        chan: rc_chan,
                        lol: (dig & 0x0C) != 0, // either bit 2 or 3 will signal a loss of link.
                    };
                    SBUS_SIGNAL.fetch_add(1, Ordering::Relaxed);
                    publish_rc(Ok(rc_packet)).await;
                } else {
                    if !valid_header {
                        SBUS_BAD_HEADER.fetch_add(1, Ordering::Relaxed);
                    }
                    if valid_header && !valid_footer {
                        SBUS_BAD_FOOTER.fetch_add(1, Ordering::Relaxed);
                    }
                }
            } else {
                SBUS_READ_ERR.fetch_add(1, Ordering::Relaxed);
            }

            if Instant::now() > timeout {
                timeout = Instant::now() + Duration::from_secs(1);
                let dig = 0x1C; // set bitfield for timeout
                SBUS_TIMEOUT.fetch_add(1, Ordering::Relaxed);
                SBUS_LAST_STATUS.store(dig as u32, Ordering::Relaxed);
                let header = packets::RosflightPacketHeader {
                    timestamp: Instant::now().as_micros(),
                    status: dig,
                };
                let rc_packet = packets::RcPacket {
                    header,
                    n_chan: 24,
                    chan: rc_chan,          // last known good values
                    lol: (dig & 0x1C) != 0, // signal a loss of link bits
                };
                SBUS_SIGNAL.fetch_add(1, Ordering::Relaxed);
                publish_rc(Ok(rc_packet)).await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut sbus: SbusRC) {
    sbus.run().await;
}
