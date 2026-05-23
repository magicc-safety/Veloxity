#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use embassy_time::Instant;
use panic_halt as _;
use pico2w::gy91::{GY91_IMU_SAMPLE_RATE_HZ, Gy91};
use rp2350_platform::hal::{
    self as rp,
    gpio::{Level, Output},
    spi::{Config as SpiConfig, Phase, Polarity, Spi},
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

fn trace(uart: &mut Uart<'_, rp::uart::Blocking>, message: &[u8]) {
    let _ = uart.blocking_write(message);
    let _ = uart.blocking_flush();
}

fn trace_hex_byte(uart: &mut Uart<'_, rp::uart::Blocking>, value: u8) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let bytes = [
        b'0',
        b'x',
        HEX[(value >> 4) as usize],
        HEX[(value & 0x0f) as usize],
        b'\r',
        b'\n',
    ];
    trace(uart, &bytes);
}

fn elapsed_us(start: Instant) -> u64 {
    start.elapsed().as_micros()
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
    trace(&mut uart, b"voloxide pico2w gy-91 spi bench\r\n");

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 1_000_000;
    spi_config.polarity = Polarity::IdleLow;
    spi_config.phase = Phase::CaptureOnFirstTransition;

    let spi = Spi::new_blocking(
        peripherals.SPI1,
        peripherals.PIN_10,
        peripherals.PIN_11,
        peripherals.PIN_12,
        spi_config,
    );
    let mut gy91 = Gy91::new(
        spi,
        Output::new(peripherals.PIN_13, Level::High),
        Output::new(peripherals.PIN_14, Level::High),
    );

    match gy91.init() {
        Ok(ids) => {
            trace(&mut uart, b"mpu whoami ");
            trace_hex_byte(&mut uart, ids.mpu);
            trace(&mut uart, b"bmp280 chipid ");
            trace_hex_byte(&mut uart, ids.bmp);
        }
        Err(err) => {
            let mut writer = UartWriter(&mut uart);
            let _ = writeln!(writer, "gy91 init failed: {:?}\r", err);
        }
    }

    let mut synthetic_us = 0_u64;
    loop {
        let raw_start = Instant::now();
        let mut raw_count = 0_u32;
        while elapsed_us(raw_start) < 1_000_000 {
            let now_us = synthetic_us + elapsed_us(raw_start);
            if gy91.sample_imu_unthrottled(now_us).is_ok() {
                raw_count = raw_count.wrapping_add(1);
            }
        }
        synthetic_us = synthetic_us.wrapping_add(1_000_000);

        let gated_start = Instant::now();
        let mut gated_calls = 0_u32;
        let mut gated_samples = 0_u32;
        while elapsed_us(gated_start) < 1_000_000 {
            gated_calls = gated_calls.wrapping_add(1);
            let now_us = synthetic_us + elapsed_us(gated_start);
            if matches!(gy91.sample_imu(now_us), Ok(Some(_))) {
                gated_samples = gated_samples.wrapping_add(1);
            }
        }
        synthetic_us = synthetic_us.wrapping_add(1_000_000);

        let mut writer = UartWriter(&mut uart);
        let _ = writeln!(
            writer,
            "raw_imu_hz={} gated_imu_hz={} gated_calls={} target_hz={}\r",
            raw_count, gated_samples, gated_calls, GY91_IMU_SAMPLE_RATE_HZ
        );
    }
}
