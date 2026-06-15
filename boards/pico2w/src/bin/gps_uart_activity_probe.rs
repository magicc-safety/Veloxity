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

fn sample_activity(pin: &Input<'static>) -> (u32, u32, u32) {
    let mut high = 0_u32;
    let mut low = 0_u32;
    let mut edges = 0_u32;
    let mut last = pin.is_high();

    for _ in 0..250_000 {
        let now = pin.is_high();
        if now {
            high += 1;
        } else {
            low += 1;
        }
        if now != last {
            edges += 1;
            last = now;
        }
        core::hint::spin_loop();
    }

    (high, low, edges)
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

    let gp8 = Input::new(peripherals.PIN_8, Pull::Down);
    let gp9 = Input::new(peripherals.PIN_9, Pull::Down);

    let _ = writeln!(writer, "veloxity pico2w gps uart activity probe");
    let _ = writeln!(writer, "activity on gp8/gp9, pulldown=internal");

    loop {
        let (gp8_high, gp8_low, gp8_edges) = sample_activity(&gp8);
        let (gp9_high, gp9_low, gp9_edges) = sample_activity(&gp9);
        let _ = writeln!(
            writer,
            "gp8 high={} low={} edges={} | gp9 high={} low={} edges={}",
            gp8_high, gp8_low, gp8_edges, gp9_high, gp9_low, gp9_edges
        );
    }
}
