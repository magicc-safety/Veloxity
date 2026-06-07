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
    for _ in 0..600_000 {
        core::hint::spin_loop();
    }
}

#[entry]
fn main() -> ! {
    let peripherals = rp::init(Default::default());

    let mut uart = Uart::new_blocking(
        peripherals.UART0,
        peripherals.PIN_0,
        peripherals.PIN_1,
        UartConfig::default(),
    );
    let mut writer = UartWriter(&mut uart);

    let sda = Input::new(peripherals.PIN_20, Pull::Up);
    let scl = Input::new(peripherals.PIN_21, Pull::Up);

    let _ = writeln!(writer, "voloxide pico2w i2c line probe");
    let _ = writeln!(writer, "gp20=sda gp21=scl pullup=internal");

    loop {
        let _ = writeln!(
            writer,
            "sda={} scl={}",
            if sda.is_high() { "high" } else { "low" },
            if scl.is_high() { "high" } else { "low" }
        );
        delay();
    }
}
