// /**
// ******************************************************************************
// * File     : vcp.rs
// * Date     : Jul 14, 2025
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

use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use rustflight_core::comm_manager::comm_link_trait::EmbeddedComInterface;
use rustflight_core::errors::SensorError;
// use defmt;

pub const VCP_TX_BUFF_SIZE: usize = 2048;
pub const VCP_RX_BUFF_SIZE: usize = 2048;

pub static VCP_TX: Pipe<CriticalSectionRawMutex, VCP_TX_BUFF_SIZE> = Pipe::new();
pub static VCP_RX: Pipe<CriticalSectionRawMutex, VCP_RX_BUFF_SIZE> = Pipe::new();

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

    pub async fn run(mut self){
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
        let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
        // Build the builder.
        let mut usb = builder.build();
        // Run the USB device.
        let usb_fut = usb.run();

        let vcp_fut = async {
            loop {
                class.wait_connection().await;
                // defmt::info!("Connected");
                let result = Self::tx_rx(&mut byte_processor, &mut class).await;
                if let Err(_) = result {
                    // defmt::warn!("VCP error");
                }
            }
        };

        join(usb_fut, vcp_fut).await;
    }

    async fn tx_rx<'d, T: Instance + 'd>(
        byte_processor: &mut ECI,
        class: &mut CdcAcmClass<'d, Driver<'d, T>>,
    ) -> Result<(), SensorError> {
        let mut tx_buf = [0u8; VCP_TX_BUFF_SIZE];
        let mut rx_buf = [0u8; VCP_RX_BUFF_SIZE];

        loop {
            let tx_fut = VCP_TX.read(&mut tx_buf);
            let rx_fut = class.read_packet(&mut rx_buf);

            match select(tx_fut, rx_fut).await {
                Either::First(n) => {
                    if n > 0 {
                        if let Err(_) = class.write_packet(&tx_buf[..n]).await {
                            // defmt::warn!("VCP TX failed, disconnecting.");
                            return Err(SensorError::GenericSensorError("VCP TX failed")); // Assume disconnect, return to outer loop
                        }
                    }
                }
                Either::Second(result) => match result {
                    Ok(n) if n > 0 => {
                        byte_processor.process_bytes(&rx_buf[..n], n).await;
                    }
                    Err(_) => {
                        // defmt::warn!("VCP RX failed, disconnecting.");
                        return Err(SensorError::GenericSensorError("VCP RX failed")); // Assume disconnect, return to outer loop
                    }
                    _ => {} // no data, do nothing
                },
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut vcp: Vcp<BasicProcessor>) {
    vcp.run().await;
}