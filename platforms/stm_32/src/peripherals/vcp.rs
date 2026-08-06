// ******************************************************************************
// * File     : platforms/stm_32/src/peripherals/vcp.rs
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

use embassy_futures::join::join;
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::usb::{Driver, Instance};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use veloxity_core::comm::interface::EmbeddedComInterface;

#[cfg(feature = "runtime-diagnostics")]
use core::sync::atomic::{AtomicU32, Ordering};

pub const VCP_TX_BUFF_SIZE: usize = 2048;
pub const VCP_RX_BUFF_SIZE: usize = 2048;
const USB_CDC_FS_PACKET_SIZE: usize = 64;

pub static VCP_TX: Pipe<CriticalSectionRawMutex, VCP_TX_BUFF_SIZE> = Pipe::new();
pub static VCP_RX: Pipe<CriticalSectionRawMutex, VCP_RX_BUFF_SIZE> = Pipe::new();

#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_FRAME_ATTEMPT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_FRAME_ENQUEUED: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_FRAME_REJECTED: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_PARTIAL_FRAME_FAILURE: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_TX_MIN_FREE: AtomicU32 = AtomicU32::new(VCP_TX_BUFF_SIZE as u32);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_IMU_ATTEMPT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_IMU_ENQUEUED: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_IMU_REJECTED: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_DEQUEUE_CALLS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_DEQUEUE_BYTES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_USB_PACKETS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_USB_BYTES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_VCP_USB_ERRORS: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "runtime-diagnostics")]
pub fn record_frame_attempt(message_id: Option<u8>) {
    VELOXITY_DIAG_VCP_FRAME_ATTEMPT.fetch_add(1, Ordering::Relaxed);
    VELOXITY_DIAG_VCP_TX_MIN_FREE.fetch_min(VCP_TX.free_capacity() as u32, Ordering::Relaxed);
    if message_id == Some(181) {
        VELOXITY_DIAG_VCP_IMU_ATTEMPT.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(feature = "runtime-diagnostics")]
pub fn record_frame_enqueued(message_id: Option<u8>) {
    VELOXITY_DIAG_VCP_FRAME_ENQUEUED.fetch_add(1, Ordering::Relaxed);
    if message_id == Some(181) {
        VELOXITY_DIAG_VCP_IMU_ENQUEUED.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(feature = "runtime-diagnostics")]
pub fn record_frame_rejected(message_id: Option<u8>, partial: bool) {
    VELOXITY_DIAG_VCP_FRAME_REJECTED.fetch_add(1, Ordering::Relaxed);
    if partial {
        VELOXITY_DIAG_VCP_PARTIAL_FRAME_FAILURE.fetch_add(1, Ordering::Relaxed);
    }
    if message_id == Some(181) {
        VELOXITY_DIAG_VCP_IMU_REJECTED.fetch_add(1, Ordering::Relaxed);
    }
}

pub struct BasicProcessor;

impl EmbeddedComInterface for BasicProcessor {
    async fn process_bytes(&mut self, buf: &[u8], num_bytes: usize) {
        VCP_RX.write_all(&buf[0..num_bytes]).await;
    }
}

pub struct Vcp<ECI: EmbeddedComInterface> {
    pub driver: Driver<'static, USB_OTG_FS>,
    pub byte_processor: ECI,
}

impl<ECI: EmbeddedComInterface> Vcp<ECI> {
    pub async fn run(self) {
        // Adapted from Embassy STM32H7 examples

        let driver = self.driver;
        let mut byte_processor = self.byte_processor;

        // Create embassy-usb Config
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB-serial example");
        config.serial_number = Some("12345678");

        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut config_descriptor = [0; 256];
        let mut bos_descriptor = [0; 256];
        let mut control_buf = [0; 64];

        let mut state = State::new();

        let mut builder = Builder::new(
            driver,
            config,
            &mut config_descriptor,
            &mut bos_descriptor,
            &mut [], // no msos descriptors
            &mut control_buf,
        );

        // Create classes on the builder.
        let class = CdcAcmClass::new(&mut builder, &mut state, 64);
        // Build the builder.
        let mut usb = builder.build();
        // Run the USB device.
        let usb_fut = usb.run();

        // Keep both USB endpoints armed independently. A single alternating RX/TX
        // loop can block RX indefinitely while it waits for outbound pipe data.
        let (mut sender, mut receiver) = class.split();
        let vcp_fut = async {
            join(
                Self::run_rx(&mut byte_processor, &mut receiver),
                Self::run_tx(&mut sender),
            )
            .await;
        };

        join(usb_fut, vcp_fut).await;
    }

    async fn run_rx<'d, T: Instance + 'd>(
        byte_processor: &mut ECI,
        receiver: &mut Receiver<'d, Driver<'d, T>>,
    ) {
        let mut rx_buf = [0u8; VCP_RX_BUFF_SIZE];

        loop {
            receiver.wait_connection().await;
            loop {
                match receiver.read_packet(&mut rx_buf).await {
                    Ok(n) if n > 0 => {
                        byte_processor.process_bytes(&rx_buf[..n], n).await;
                    }
                    Ok(_) => {}
                    Err(_) => break,
                }
            }
        }
    }

    async fn run_tx<'d, T: Instance + 'd>(sender: &mut Sender<'d, Driver<'d, T>>) {
        let mut tx_buf = [0u8; VCP_TX_BUFF_SIZE];

        loop {
            sender.wait_connection().await;
            'connected: loop {
                let n = VCP_TX.read(&mut tx_buf).await;
                #[cfg(feature = "runtime-diagnostics")]
                {
                    VELOXITY_DIAG_VCP_DEQUEUE_CALLS.fetch_add(1, Ordering::Relaxed);
                    VELOXITY_DIAG_VCP_DEQUEUE_BYTES.fetch_add(n as u32, Ordering::Relaxed);
                }
                for packet in tx_buf[..n].chunks(USB_CDC_FS_PACKET_SIZE) {
                    if sender.write_packet(packet).await.is_err() {
                        #[cfg(feature = "runtime-diagnostics")]
                        VELOXITY_DIAG_VCP_USB_ERRORS.fetch_add(1, Ordering::Relaxed);
                        break 'connected;
                    }
                    #[cfg(feature = "runtime-diagnostics")]
                    {
                        VELOXITY_DIAG_VCP_USB_PACKETS.fetch_add(1, Ordering::Relaxed);
                        VELOXITY_DIAG_VCP_USB_BYTES
                            .fetch_add(packet.len() as u32, Ordering::Relaxed);
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(vcp: Vcp<BasicProcessor>) {
    vcp.run().await;
}
