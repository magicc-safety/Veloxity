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

fn delay_short() {
    for _ in 0..80 {
        core::hint::spin_loop();
    }
}

fn delay_long() {
    for _ in 0..700_000 {
        core::hint::spin_loop();
    }
}

fn set_pin(pin: &mut Output<'static>, high: bool) {
    if high {
        pin.set_high();
    } else {
        pin.set_low();
    }
}

fn transfer_byte(
    sck: &mut Output<'static>,
    mosi: &mut Output<'static>,
    miso: &Input<'static>,
    byte: u8,
    cpol: bool,
    cpha: bool,
) -> u8 {
    let active = !cpol;
    let idle = cpol;
    let mut rx = 0_u8;

    for bit in (0..8).rev() {
        if !cpha {
            set_pin(mosi, ((byte >> bit) & 1) != 0);
            delay_short();
            set_pin(sck, active);
            delay_short();
            rx = (rx << 1) | u8::from(miso.is_high());
            set_pin(sck, idle);
            delay_short();
        } else {
            set_pin(sck, active);
            set_pin(mosi, ((byte >> bit) & 1) != 0);
            delay_short();
            set_pin(sck, idle);
            delay_short();
            rx = (rx << 1) | u8::from(miso.is_high());
        }
    }

    rx
}

fn read_id(
    sck: &mut Output<'static>,
    mosi: &mut Output<'static>,
    miso: &Input<'static>,
    cs: &mut Output<'static>,
    cpol: bool,
    cpha: bool,
) -> u8 {
    set_pin(sck, cpol);
    cs.set_low();
    delay_short();
    let _ = transfer_byte(sck, mosi, miso, 0xd0 | 0x80, cpol, cpha);
    let id = transfer_byte(sck, mosi, miso, 0x00, cpol, cpha);
    cs.set_high();
    set_pin(sck, cpol);
    id
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

    let mut sck = Output::new(peripherals.PIN_10, Level::Low);
    let mut mosi = Output::new(peripherals.PIN_11, Level::Low);
    let miso = Input::new(peripherals.PIN_12, Pull::Up);
    let mut cs14 = Output::new(peripherals.PIN_14, Level::High);
    let mut cs15 = Output::new(peripherals.PIN_15, Level::High);
    let _imu_cs = Output::new(peripherals.PIN_13, Level::High);

    let _ = writeln!(writer, "veloxity pico2w bmp280 bitbang spi scan");
    let _ = writeln!(writer, "sck=gp10 mosi=gp11 miso=gp12 cs=gp14/gp15");

    loop {
        cs14.set_high();
        cs15.set_high();
        for mode in 0..4 {
            let cpol = (mode & 0b10) != 0;
            let cpha = (mode & 0b01) != 0;
            let id14 = read_id(&mut sck, &mut mosi, &miso, &mut cs14, cpol, cpha);
            let id15 = read_id(&mut sck, &mut mosi, &miso, &mut cs15, cpol, cpha);
            let _ = writeln!(
                writer,
                "mode{} cpol={} cpha={} cs14=0x{:02x} cs15=0x{:02x}",
                mode,
                u8::from(cpol),
                u8::from(cpha),
                id14,
                id15
            );
        }
        let _ = writeln!(
            writer,
            "miso_idle={}",
            if miso.is_high() { "high" } else { "low" }
        );
        delay_long();
    }
}
