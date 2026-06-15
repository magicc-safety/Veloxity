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

const BMP280_ID_REG: u8 = 0xd0;
const BMP280_CALIB_REG: u8 = 0x88;
const BMP280_CTRL_MEAS_REG: u8 = 0xf4;
const BMP280_CONFIG_REG: u8 = 0xf5;
const BMP280_PRESS_REG: u8 = 0xf7;

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

#[derive(Default)]
struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
}

fn delay() {
    for _ in 0..400_000 {
        core::hint::spin_loop();
    }
}

fn read_u8(
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
    addr: u8,
    reg: u8,
) -> Result<u8, ()> {
    let mut out = [0_u8; 1];
    i2c.blocking_write_read(addr, &[reg], &mut out)
        .map_err(|_| ())?;
    Ok(out[0])
}

fn write_u8(
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
    addr: u8,
    reg: u8,
    value: u8,
) -> Result<(), ()> {
    i2c.blocking_write(addr, &[reg, value]).map_err(|_| ())
}

fn read_calibration(
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
    addr: u8,
) -> Result<Calibration, ()> {
    let mut raw = [0_u8; 24];
    i2c.blocking_write_read(addr, &[BMP280_CALIB_REG], &mut raw)
        .map_err(|_| ())?;
    Ok(Calibration {
        dig_t1: u16_le(raw[0], raw[1]),
        dig_t2: i16_le(raw[2], raw[3]),
        dig_t3: i16_le(raw[4], raw[5]),
        dig_p1: u16_le(raw[6], raw[7]),
        dig_p2: i16_le(raw[8], raw[9]),
        dig_p3: i16_le(raw[10], raw[11]),
        dig_p4: i16_le(raw[12], raw[13]),
        dig_p5: i16_le(raw[14], raw[15]),
        dig_p6: i16_le(raw[16], raw[17]),
        dig_p7: i16_le(raw[18], raw[19]),
        dig_p8: i16_le(raw[20], raw[21]),
        dig_p9: i16_le(raw[22], raw[23]),
    })
}

fn read_sample(
    i2c: &mut I2c<'static, rp::peripherals::I2C0, rp::i2c::Blocking>,
    addr: u8,
    cal: &Calibration,
) -> Result<(f32, f32), ()> {
    let mut raw = [0_u8; 6];
    i2c.blocking_write_read(addr, &[BMP280_PRESS_REG], &mut raw)
        .map_err(|_| ())?;
    let adc_p = ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);
    let adc_t = ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);
    Ok(compensate(cal, adc_p, adc_t))
}

fn compensate(cal: &Calibration, adc_p: i32, adc_t: i32) -> (f32, f32) {
    let var1 = (((adc_t >> 3) - ((cal.dig_t1 as i32) << 1)) * cal.dig_t2 as i32) >> 11;
    let var2 = (((((adc_t >> 4) - cal.dig_t1 as i32) * ((adc_t >> 4) - cal.dig_t1 as i32)) >> 12)
        * cal.dig_t3 as i32)
        >> 14;
    let t_fine = var1 + var2;
    let temperature = ((t_fine * 5 + 128) >> 8) as f32 / 100.0;

    let mut p_var1 = t_fine as i64 - 128000;
    let mut p_var2 = p_var1 * p_var1 * cal.dig_p6 as i64;
    p_var2 += (p_var1 * cal.dig_p5 as i64) << 17;
    p_var2 += (cal.dig_p4 as i64) << 35;
    p_var1 = ((p_var1 * p_var1 * cal.dig_p3 as i64) >> 8) + ((p_var1 * cal.dig_p2 as i64) << 12);
    p_var1 = (((1_i64 << 47) + p_var1) * cal.dig_p1 as i64) >> 33;
    if p_var1 == 0 {
        return (0.0, temperature);
    }
    let mut pressure = 1048576 - adc_p as i64;
    pressure = (((pressure << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (cal.dig_p9 as i64 * (pressure >> 13) * (pressure >> 13)) >> 25;
    p_var2 = (cal.dig_p8 as i64 * pressure) >> 19;
    pressure = ((pressure + p_var1 + p_var2) >> 8) + ((cal.dig_p7 as i64) << 4);
    (pressure as f32 / 256.0, temperature)
}

fn u16_le(lo: u8, hi: u8) -> u16 {
    u16::from_le_bytes([lo, hi])
}

fn i16_le(lo: u8, hi: u8) -> i16 {
    i16::from_le_bytes([lo, hi])
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
    let _ = writeln!(writer, "veloxity pico2w bmp280 i2c probe");
    let _ = writeln!(writer, "i2c0 sda=gp20 scl=gp21");

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 10_000;
    i2c_config.sda_pullup = true;
    i2c_config.scl_pullup = true;

    let mut i2c = I2c::new_blocking(
        peripherals.I2C0,
        peripherals.PIN_21,
        peripherals.PIN_20,
        i2c_config,
    );

    for candidate in [0x68_u8, 0x69_u8] {
        match read_u8(&mut i2c, candidate, 0x75) {
            Ok(id) => {
                let _ = writeln!(writer, "mpu addr=0x{:02x} whoami=0x{:02x}", candidate, id);
            }
            Err(()) => {
                let _ = writeln!(writer, "mpu addr=0x{:02x} no ack", candidate);
            }
        }
    }

    let mut addr = None;
    for candidate in [0x76_u8, 0x77_u8] {
        match read_u8(&mut i2c, candidate, BMP280_ID_REG) {
            Ok(id) => {
                let _ = writeln!(writer, "addr=0x{:02x} chipid=0x{:02x}", candidate, id);
                if id == 0x58 {
                    addr = Some(candidate);
                    break;
                }
            }
            Err(()) => {
                let _ = writeln!(writer, "addr=0x{:02x} no ack", candidate);
            }
        }
    }

    let Some(addr) = addr else {
        let _ = writeln!(writer, "bmp280 not found");
        loop {
            delay();
        }
    };

    let cal = match read_calibration(&mut i2c, addr) {
        Ok(cal) => cal,
        Err(()) => {
            let _ = writeln!(writer, "calibration read failed");
            loop {
                delay();
            }
        }
    };
    let _ = writeln!(
        writer,
        "cal t1={} t2={} t3={} p1={}",
        cal.dig_t1, cal.dig_t2, cal.dig_t3, cal.dig_p1
    );

    let _ = write_u8(&mut i2c, addr, BMP280_CONFIG_REG, 0xa0);
    let _ = write_u8(&mut i2c, addr, BMP280_CTRL_MEAS_REG, 0x4f);

    loop {
        match read_sample(&mut i2c, addr, &cal) {
            Ok((pressure, temperature)) => {
                let _ = writeln!(
                    writer,
                    "baro pressure={:.1} temp={:.2}",
                    pressure, temperature
                );
            }
            Err(()) => {
                let _ = writeln!(writer, "baro read failed");
            }
        }
        delay();
    }
}
