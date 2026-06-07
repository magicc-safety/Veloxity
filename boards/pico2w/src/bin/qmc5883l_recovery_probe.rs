#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use panic_halt as _;
use rp2350_platform::hal::{
    self as rp,
    gpio::{Input, Level, Output, Pull},
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

fn short_delay() {
    for _ in 0..8_000 {
        core::hint::spin_loop();
    }
}

fn long_delay() {
    for _ in 0..700_000 {
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

    let sda_in = Input::new(peripherals.PIN_20, Pull::Up);
    let mut scl_out = Output::new(peripherals.PIN_21, Level::High);

    let _ = writeln!(writer, "voloxide pico2w qmc5883l recovery probe");
    let _ = writeln!(writer, "i2c recovery sda=gp20 scl=gp21");
    let _ = writeln!(
        writer,
        "before recovery sda={} scl=high",
        if sda_in.is_high() { "high" } else { "low" }
    );

    for _ in 0..18 {
        scl_out.set_low();
        short_delay();
        scl_out.set_high();
        short_delay();
    }

    let _ = writeln!(
        writer,
        "after recovery sda={} scl=high",
        if sda_in.is_high() { "high" } else { "low" }
    );

    loop {
        let _ = writeln!(
            writer,
            "line state sda={} scl=high",
            if sda_in.is_high() { "high" } else { "low" }
        );
        long_delay();
    }
}
