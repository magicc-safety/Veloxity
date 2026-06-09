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

fn delay() {
    for _ in 0..700_000 {
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

    let miso = Input::new(peripherals.PIN_12, Pull::Up);
    let mut bmp_cs = Output::new(peripherals.PIN_15, Level::High);
    let _imu_cs = Output::new(peripherals.PIN_13, Level::High);

    let _ = writeln!(writer, "voloxide pico2w bmp280 spi line probe");
    let _ = writeln!(
        writer,
        "gp12=miso input pullup gp15=bmp_cs output gp13=imu_cs high"
    );

    loop {
        bmp_cs.set_high();
        delay();
        let high_selected = miso.is_high();

        bmp_cs.set_low();
        delay();
        let low_selected = miso.is_high();

        bmp_cs.set_high();
        let _ = writeln!(
            writer,
            "miso cs_high={} cs_low={}",
            if high_selected { "high" } else { "low" },
            if low_selected { "high" } else { "low" }
        );
        delay();
    }
}
