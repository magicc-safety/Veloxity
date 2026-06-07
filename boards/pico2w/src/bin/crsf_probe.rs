#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use embedded_hal_nb::serial::Read as _;
use panic_halt as _;
use pico2w::rc_receiver::{CRSF_BAUDRATE, CrsfRcParser};
use rp2350_platform::hal::{
    self as rp,
    uart::{Config as UartConfig, Uart},
};

struct UartWriter<'a>(&'a mut Uart<'static, rp::uart::Blocking>);

impl Write for UartWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.0
            .blocking_write(s.as_bytes())
            .map_err(|_| fmt::Error)?;
        self.0.blocking_flush().map_err(|_| fmt::Error)?;
        Ok(())
    }
}

fn delay_window() {
    for _ in 0..300_000 {
        core::hint::spin_loop();
    }
}

#[entry]
fn main() -> ! {
    let peripherals = rp::init(Default::default());

    let mut debug_uart = Uart::new_blocking(
        peripherals.UART0,
        peripherals.PIN_0,
        peripherals.PIN_1,
        UartConfig::default(),
    );
    let mut writer = UartWriter(&mut debug_uart);

    let mut crsf_config = UartConfig::default();
    crsf_config.baudrate = CRSF_BAUDRATE;
    let mut crsf_uart = Uart::new_blocking(
        peripherals.UART1,
        peripherals.PIN_8,
        peripherals.PIN_9,
        crsf_config,
    );

    let _ = writeln!(writer, "voloxide pico2w crsf probe");
    let _ = writeln!(
        writer,
        "uart1 tx=gp8 rx=gp9 baud={} expect receiver tx -> gp9 rx -> gp8",
        CRSF_BAUDRATE
    );

    let mut parser = CrsfRcParser::new();
    let mut now_us = 0_u64;
    let mut total_bytes = 0_u32;
    let mut frame_count = 0_u32;

    loop {
        let mut window_bytes = 0_u32;
        let mut latest = None;

        for _ in 0..25_000 {
            match crsf_uart.read() {
                Ok(byte) => {
                    total_bytes = total_bytes.wrapping_add(1);
                    window_bytes = window_bytes.wrapping_add(1);
                    now_us = now_us.wrapping_add(24);
                    if let Some(packet) = parser.push_bytes(&[byte], now_us) {
                        frame_count = frame_count.wrapping_add(1);
                        latest = Some(packet);
                    }
                }
                Err(nb::Error::WouldBlock) => {}
                Err(nb::Error::Other(_)) => {}
            }
            core::hint::spin_loop();
        }

        if let Some(packet) = latest {
            let _ = writeln!(
                writer,
                "frames={} ch1={:.3} ch2={:.3} ch3={:.3} ch4={:.3} bytes={}",
                frame_count,
                packet.chan[0],
                packet.chan[1],
                packet.chan[2],
                packet.chan[3],
                window_bytes,
            );
        } else {
            let _ = writeln!(
                writer,
                "crsf bytes={} total={} frames={}",
                window_bytes, total_bytes, frame_count
            );
        }

        delay_window();
    }
}
