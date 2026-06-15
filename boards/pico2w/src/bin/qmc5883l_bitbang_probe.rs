#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use panic_halt as _;
use rp2350_platform::hal::{
    self as rp,
    gpio::{Flex, Pull},
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
    for _ in 0..3_000 {
        core::hint::spin_loop();
    }
}

fn long_delay() {
    for _ in 0..700_000 {
        core::hint::spin_loop();
    }
}

fn release(pin: &mut Flex<'static>) {
    pin.set_as_input();
}

fn drive_low(pin: &mut Flex<'static>) {
    pin.set_low();
    pin.set_as_output();
}

fn scl_high(scl: &mut Flex<'static>) {
    release(scl);
    delay();
}

fn scl_low(scl: &mut Flex<'static>) {
    drive_low(scl);
    delay();
}

fn start(sda: &mut Flex<'static>, scl: &mut Flex<'static>) {
    release(sda);
    scl_high(scl);
    drive_low(sda);
    delay();
    scl_low(scl);
}

fn stop(sda: &mut Flex<'static>, scl: &mut Flex<'static>) {
    drive_low(sda);
    delay();
    scl_high(scl);
    release(sda);
    delay();
}

fn write_byte(sda: &mut Flex<'static>, scl: &mut Flex<'static>, byte: u8) -> bool {
    for bit in (0..8).rev() {
        if ((byte >> bit) & 1) == 0 {
            drive_low(sda);
        } else {
            release(sda);
        }
        scl_high(scl);
        scl_low(scl);
    }

    release(sda);
    scl_high(scl);
    let ack = sda.is_low();
    scl_low(scl);
    ack
}

fn read_byte(sda: &mut Flex<'static>, scl: &mut Flex<'static>, ack: bool) -> u8 {
    let mut byte = 0_u8;
    release(sda);
    for _ in 0..8 {
        byte <<= 1;
        scl_high(scl);
        if sda.is_high() {
            byte |= 1;
        }
        scl_low(scl);
    }

    if ack {
        drive_low(sda);
    } else {
        release(sda);
    }
    scl_high(scl);
    scl_low(scl);
    release(sda);
    byte
}

fn write_reg(sda: &mut Flex<'static>, scl: &mut Flex<'static>, reg: u8, value: u8) -> bool {
    start(sda, scl);
    let a0 = write_byte(sda, scl, QMC5883L_ADDR << 1);
    let a1 = write_byte(sda, scl, reg);
    let a2 = write_byte(sda, scl, value);
    stop(sda, scl);
    a0 && a1 && a2
}

fn read_regs(sda: &mut Flex<'static>, scl: &mut Flex<'static>, reg: u8, out: &mut [u8]) -> bool {
    start(sda, scl);
    let a0 = write_byte(sda, scl, QMC5883L_ADDR << 1);
    let a1 = write_byte(sda, scl, reg);
    start(sda, scl);
    let a2 = write_byte(sda, scl, (QMC5883L_ADDR << 1) | 1);
    if !(a0 && a1 && a2) {
        stop(sda, scl);
        return false;
    }
    let last = out.len().saturating_sub(1);
    for (idx, byte) in out.iter_mut().enumerate() {
        *byte = read_byte(sda, scl, idx != last);
    }
    stop(sda, scl);
    true
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

    let mut sda = Flex::new(peripherals.PIN_20);
    let mut scl = Flex::new(peripherals.PIN_21);
    sda.set_pull(Pull::Up);
    scl.set_pull(Pull::Up);
    release(&mut sda);
    release(&mut scl);

    let _ = writeln!(writer, "veloxity pico2w qmc5883l bitbang probe");
    let _ = writeln!(writer, "bitbang i2c sda=gp20 scl=gp21 addr=0x0d");
    let _ = writeln!(
        writer,
        "idle sda={} scl={}",
        if sda.is_high() { "high" } else { "low" },
        if scl.is_high() { "high" } else { "low" }
    );

    let mut status = [0_u8; 1];
    if read_regs(&mut sda, &mut scl, 0x06, &mut status) {
        let _ = writeln!(writer, "qmc5883l ack status=0x{:02x}", status[0]);
    } else {
        let _ = writeln!(writer, "qmc5883l no ack");
        loop {
            long_delay();
        }
    }

    let _ = write_reg(&mut sda, &mut scl, 0x0b, 0x01);
    let _ = write_reg(&mut sda, &mut scl, 0x09, 0x1d);

    loop {
        let mut raw = [0_u8; 6];
        let mut status = [0_u8; 1];
        if read_regs(&mut sda, &mut scl, 0x06, &mut status)
            && read_regs(&mut sda, &mut scl, 0x00, &mut raw)
        {
            let x = i16::from_le_bytes([raw[0], raw[1]]);
            let y = i16::from_le_bytes([raw[2], raw[3]]);
            let z = i16::from_le_bytes([raw[4], raw[5]]);
            let _ = writeln!(
                writer,
                "mag status=0x{:02x} x={} y={} z={}",
                status[0], x, y, z
            );
        } else {
            let _ = writeln!(writer, "mag read failed");
        }
        long_delay();
    }
}
