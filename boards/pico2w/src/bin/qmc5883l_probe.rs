#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use panic_halt as _;
use rp2350_platform::hal::{
    self as rp,
    i2c::{Config as I2cConfig, I2c},
    uart::{Config as UartConfig, Uart},
};

const QMC5883L_ADDR: u8 = 0x0d;

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

fn read_reg(
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
    reg: u8,
    out: &mut [u8],
) -> Result<(), ()> {
    i2c.blocking_write_read(QMC5883L_ADDR, &[reg], out)
        .map_err(|_| ())
}

fn write_reg(
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
    reg: u8,
    value: u8,
) -> Result<(), ()> {
    i2c.blocking_write(QMC5883L_ADDR, &[reg, value])
        .map_err(|_| ())
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

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 100_000;
    i2c_config.sda_pullup = true;
    i2c_config.scl_pullup = true;
    let mut i2c = I2c::new_blocking(
        peripherals.I2C0,
        peripherals.PIN_21,
        peripherals.PIN_20,
        i2c_config,
    );

    let _ = writeln!(writer, "voloxide pico2w qmc5883l probe");
    let _ = writeln!(writer, "i2c0 sda=gp20 scl=gp21 addr=0x0d");

    let mut status = [0_u8; 1];
    match read_reg(&mut i2c, 0x06, &mut status) {
        Ok(()) => {
            let _ = writeln!(writer, "qmc5883l ack status=0x{:02x}", status[0]);
        }
        Err(()) => {
            let _ = writeln!(writer, "qmc5883l no ack/read failed");
            loop {
                delay();
            }
        }
    }

    let _ = write_reg(&mut i2c, 0x0b, 0x01);
    let _ = write_reg(&mut i2c, 0x09, 0x1d);

    loop {
        let mut raw = [0_u8; 6];
        let mut status = [0_u8; 1];
        match (
            read_reg(&mut i2c, 0x06, &mut status),
            read_reg(&mut i2c, 0x00, &mut raw),
        ) {
            (Ok(()), Ok(())) => {
                let x = i16::from_le_bytes([raw[0], raw[1]]);
                let y = i16::from_le_bytes([raw[2], raw[3]]);
                let z = i16::from_le_bytes([raw[4], raw[5]]);
                let _ = writeln!(
                    writer,
                    "mag status=0x{:02x} x={} y={} z={}",
                    status[0], x, y, z
                );
            }
            _ => {
                let _ = writeln!(writer, "mag read failed");
            }
        }
        delay();
    }
}
