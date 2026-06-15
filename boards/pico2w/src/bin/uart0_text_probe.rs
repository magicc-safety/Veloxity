#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use panic_halt as _;
use rp2350_platform::hal::{
    self as rp,
    uart::{Config as UartConfig, Uart},
};

const UART_BAUD: u32 = 2_000_000;

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

fn delay() {
    for _ in 0..1_000_000 {
        core::hint::spin_loop();
    }
}

#[entry]
fn main() -> ! {
    let peripherals = rp::init(Default::default());

    let mut config = UartConfig::default();
    config.baudrate = UART_BAUD;

    let mut uart = Uart::new_blocking(
        peripherals.UART0,
        peripherals.PIN_0,
        peripherals.PIN_1,
        config,
    );
    let mut writer = UartWriter(&mut uart);

    let _ = writeln!(writer, "veloxity pico2w uart0 text probe");
    let _ = writeln!(writer, "uart0 tx=gp0 rx=gp1 baud={}", UART_BAUD);

    let mut count = 0_u32;
    loop {
        let _ = writeln!(writer, "PICO_UART_TEST {}", count);
        count = count.wrapping_add(1);
        delay();
    }
}
