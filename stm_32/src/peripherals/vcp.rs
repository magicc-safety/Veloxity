//#![allow(unused)]

use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::peripherals::USB_OTG_FS;
use defmt::{panic, *};
use embassy_futures::join::join;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;

use rustflight_core::comm_manager::comm_link_trait::EmbeddedComInterface;
use rustflight_core::errors::{self, SensorError};

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
                info!("Connected");
                let result = Self::tx_rx(&mut byte_processor, &mut class).await;
                if let Err(_) = result {
                    warn!("VCP error");
                }
            }
        };

        join(usb_fut, vcp_fut).await;
    }

    async fn tx_rx<'d, T: Instance + 'd>(
        byte_processor: &mut ECI, 
        class: &mut CdcAcmClass<'d, Driver<'d, T>>,
    ) -> Result<(), SensorError> {
        const MAX_PACKET_SIZE: usize = 64;
        let mut tx_buf = [0u8; VCP_TX_BUFF_SIZE];
        let mut rx_buf = [0u8; VCP_RX_BUFF_SIZE];
        loop {
            // tx
            // defmt::debug!("Waiting for data to send...");
            let n = VCP_TX.read(&mut tx_buf).await;
            // defmt::debug!("Read {} bytes from VCP_TX", n);
            if n > 0 {
                let data_to_send = &tx_buf[..n];
                for chunk in data_to_send.chunks(MAX_PACKET_SIZE) {
                    // defmt::debug!("Sending {} bytes to USB host", chunk.len());
                    let result = class
                        .write_packet(chunk)
                        .await
                        .map_err(|e| match e {
                            _ => errors::TelemError::GenericTelemError("UsbTx failed!"),
                        });
                }
            }
            // rx
            // defmt::debug!("Waiting for data from USB host...");
            let result = class.read_packet(&mut rx_buf).await;
            match result {
                Err(_) => {
                    // defmt::warn!("System: Usb read error");
                    Timer::after_millis(1).await;
                },
                Ok(n) => {
                    // defmt::debug!("Received {} bytes from USB host", n);
                    if n > 0 && n <= VCP_RX_BUFF_SIZE {
                        byte_processor.process_bytes(&rx_buf[..n], n).await;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut vcp: Vcp<BasicProcessor>) {
    vcp.run().await;
}