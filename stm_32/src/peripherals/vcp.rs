//#![allow(unused)]

use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::peripherals::USB_OTG_FS;
use defmt::{panic, *};
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;

pub struct Vcp {
    pub driver: Driver<'static, USB_OTG_FS>,
}

impl Vcp {

    pub async fn run(self){
        // Adapted from Embassy STM32H7 examples

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
            self.driver,
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

        // Do stuff with the class!
        let echo_fut = async {
            loop {
                class.wait_connection().await;
                info!("Connected");
                let _ = echo(&mut class).await;
                info!("Disconnected");
            }
        };

        // Run everything concurrently.
        // If we had made everything `'static` above instead, we could do this using separate tasks instead.
        join(usb_fut, echo_fut).await;
    }
}

#[embassy_executor::task]
pub async fn task(mut vcp: Vcp ) {
    vcp.run().await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        // TODO: map to rustflight core errors
        let n = match class.read_packet(&mut buf).await {
            Ok(n) => n, 
            Err(e) => { 
                match e {
                    EndpointError::BufferOverflow => warn!("USB read buffer overflow!"),
                    EndpointError::Disabled => info!("USB endpoint disabled."),
                }
                return Err(Disconnected {});
            }
        };

        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}