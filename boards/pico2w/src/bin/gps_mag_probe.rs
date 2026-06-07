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
    for _ in 0..500_000 {
        core::hint::spin_loop();
    }
}

fn i2c_read(
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
    addr: u8,
    reg: u8,
    out: &mut [u8],
) -> Result<(), ()> {
    i2c.blocking_write_read(addr, &[reg], out).map_err(|_| ())
}

fn i2c_write(
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
    addr: u8,
    bytes: &[u8],
) -> Result<(), ()> {
    i2c.blocking_write(addr, bytes).map_err(|_| ())
}

fn read_qmc5883l(
    writer: &mut UartWriter<'_>,
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
) {
    let _ = i2c_write(i2c, QMC5883L_ADDR, &[0x0b, 0x01]);
    let _ = i2c_write(i2c, QMC5883L_ADDR, &[0x09, 0x1d]);
    delay();

    let mut status = [0_u8; 1];
    let mut raw = [0_u8; 6];
    match (
        i2c_read(i2c, QMC5883L_ADDR, 0x06, &mut status),
        i2c_read(i2c, QMC5883L_ADDR, 0x00, &mut raw),
    ) {
        (Ok(()), Ok(())) => {
            let x = i16::from_le_bytes([raw[0], raw[1]]);
            let y = i16::from_le_bytes([raw[2], raw[3]]);
            let z = i16::from_le_bytes([raw[4], raw[5]]);
            let _ = writeln!(
                writer,
                "qmc5883l status=0x{:02x} x={} y={} z={}",
                status[0], x, y, z
            );
        }
        _ => {
            let _ = writeln!(writer, "qmc5883l read failed");
        }
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
        peripherals.PIN_4,
        peripherals.PIN_5,
        gps_config,
    );

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

    let _ = writeln!(writer, "voloxide pico2w gps+mag probe");
    let _ = writeln!(writer, "gps uart1 tx=gp4 rx=gp5 baud=115200");
    let _ = writeln!(writer, "mag i2c0 sda=gp20 scl=gp21");

    let mut one = [0_u8; 1];
    if i2c_read(&mut i2c, QMC5883L_ADDR, 0x00, &mut one).is_ok() {
        let _ = writeln!(writer, "qmc5883l addr=0x0d ack");
        read_qmc5883l(&mut writer, &mut i2c);
    } else {
        let _ = writeln!(writer, "qmc5883l addr=0x0d no ack");
    }

    let _ = writeln!(writer, "gps uart bytes:");
    loop {
        let mut byte = [0_u8; 1];
        if gps_uart.blocking_read(&mut byte).is_ok() {
            write_byte_repr(&mut writer, byte[0]);
        } else {
            let _ = writeln!(writer, "gps uart read error");
            delay();
        }
    }
}
