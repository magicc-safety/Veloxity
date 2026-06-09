#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use embedded_hal_nb::serial::Read as _;
use panic_halt as _;
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

fn write_byte_repr(writer: &mut UartWriter<'_>, byte: u8) {
    match byte {
        b'\r' => {
            let _ = write!(writer, "\\r");
        }
        b'\n' => {
            let _ = writeln!(writer, "\\n");
        }
        0x20..=0x7e => {
            let _ = write!(writer, "{}", byte as char);
        }
        _ => {
            let _ = write!(writer, "<{:02x}>", byte);
        }
    }
}

fn delay_scan_window() {
    for _ in 0..2_500 {
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

    let mut gps_config = UartConfig::default();
    gps_config.baudrate = 115_200;
    let mut gps_uart = Uart::new_blocking(
        peripherals.UART1,
        peripherals.PIN_8,
        peripherals.PIN_9,
        gps_config,
    );

    let _ = writeln!(writer, "voloxide pico2w gps uart probe");
    let _ = writeln!(writer, "gps uart1 tx=gp8 rx=gp9 baud=115200");
    let _ = writeln!(writer, "expect m100 tx -> gp9, m100 rx -> gp8");

    loop {
        for baudrate in [
            4_800_u32, 9_600, 19_200, 38_400, 57_600, 115_200, 230_400, 460_800,
        ] {
            gps_uart.set_baudrate(baudrate);
            let _ = writeln!(writer, "gps baud {}:", baudrate);

            let mut count = 0_u32;
            for _ in 0..700 {
                match gps_uart.read() {
                    Ok(byte) => {
                        count += 1;
                        if count <= 220 {
                            write_byte_repr(&mut writer, byte);
                        }
                    }
                    Err(nb::Error::WouldBlock) => {}
                    Err(nb::Error::Other(_)) => {
                        let _ = write!(writer, "<err>");
                    }
                }
                delay_scan_window();
            }

            if count == 0 {
                let _ = writeln!(writer, "no bytes");
            } else {
                let _ = writeln!(writer, "\nbytes={}", count);
            }
        }
    }
}
