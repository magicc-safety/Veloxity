#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use panic_halt as _;
use rp2350_platform::hal::{
    self as rp,
    gpio::{Input, Pull},
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

fn delay() {
    for _ in 0..500_000 {
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

    let gp4 = Input::new(peripherals.PIN_4, Pull::Down);
    let gp5 = Input::new(peripherals.PIN_5, Pull::Down);

    let _ = writeln!(writer, "veloxity pico2w gps uart line probe");
    let _ = writeln!(
        writer,
        "gp4=m100 rx candidate, gp5=m100 tx candidate, pulldown=internal"
    );

    loop {
        let _ = writeln!(
            writer,
            "gp4={} gp5={}",
            if gp4.is_high() { "high" } else { "low" },
            if gp5.is_high() { "high" } else { "low" }
        );
        delay();
    }
}
