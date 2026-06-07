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

fn delay_short() {
    for _ in 0..120 {
        core::hint::spin_loop();
    }
}

fn delay_long() {
    for _ in 0..700_000 {
        core::hint::spin_loop();
    }
}

fn transfer_byte(
    sck: &mut Output<'static>,
    mosi: &mut Output<'static>,
    miso: &Input<'static>,
    byte: u8,
) -> u8 {
    let mut rx = 0_u8;
    for bit in (0..8).rev() {
        if ((byte >> bit) & 1) != 0 {
            mosi.set_high();
        } else {
            mosi.set_low();
        }
        delay_short();
        sck.set_high();
        delay_short();
        rx = (rx << 1) | u8::from(miso.is_high());
        sck.set_low();
        delay_short();
    }
    rx
}

fn read_reg(
    sck: &mut Output<'static>,
    mosi: &mut Output<'static>,
    miso: &Input<'static>,
    cs: &mut Output<'static>,
    reg: u8,
) -> u8 {
    cs.set_low();
    delay_short();
    let _ = transfer_byte(sck, mosi, miso, reg | 0x80);
    let value = transfer_byte(sck, mosi, miso, 0);
    cs.set_high();
    value
}

fn read_regs(
    sck: &mut Output<'static>,
    mosi: &mut Output<'static>,
    miso: &Input<'static>,
    cs: &mut Output<'static>,
    reg: u8,
    out: &mut [u8],
) {
    cs.set_low();
    delay_short();
    let _ = transfer_byte(sck, mosi, miso, reg | 0x80);
    for byte in out {
        *byte = transfer_byte(sck, mosi, miso, 0);
    }
    cs.set_high();
}

fn write_reg(
    sck: &mut Output<'static>,
    mosi: &mut Output<'static>,
    miso: &Input<'static>,
    cs: &mut Output<'static>,
    reg: u8,
    value: u8,
) {
    cs.set_low();
    delay_short();
    let _ = transfer_byte(sck, mosi, miso, reg & 0x7f);
    let _ = transfer_byte(sck, mosi, miso, value);
    cs.set_high();
}

fn read_calibration(
    sck: &mut Output<'static>,
    mosi: &mut Output<'static>,
    miso: &Input<'static>,
    cs: &mut Output<'static>,
) -> Calibration {
    let mut raw = [0_u8; 24];
    read_regs(sck, mosi, miso, cs, BMP280_CALIB_REG, &mut raw);
    Calibration {
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
    }
}

fn read_sample(
    sck: &mut Output<'static>,
    mosi: &mut Output<'static>,
    miso: &Input<'static>,
    cs: &mut Output<'static>,
    cal: &Calibration,
) -> (f32, f32) {
    let mut raw = [0_u8; 6];
    read_regs(sck, mosi, miso, cs, BMP280_PRESS_REG, &mut raw);
    let adc_p = ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);
    let adc_t = ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);
    compensate(cal, adc_p, adc_t)
}

fn compensate(cal: &Calibration, adc_p: i32, adc_t: i32) -> (f32, f32) {
    let var1 = (((adc_t >> 3) - ((cal.dig_t1 as i32) << 1)) * cal.dig_t2 as i32) >> 11;
    let var2 = (((((adc_t >> 4) - cal.dig_t1 as i32)
        * ((adc_t >> 4) - cal.dig_t1 as i32))
        >> 12)
        * cal.dig_t3 as i32)
        >> 14;
    let t_fine = var1 + var2;
    let temperature = ((t_fine * 5 + 128) >> 8) as f32 / 100.0;

    let mut p_var1 = t_fine as i64 - 128000;
    let mut p_var2 = p_var1 * p_var1 * cal.dig_p6 as i64;
    p_var2 += (p_var1 * cal.dig_p5 as i64) << 17;
    p_var2 += (cal.dig_p4 as i64) << 35;
    p_var1 =
        ((p_var1 * p_var1 * cal.dig_p3 as i64) >> 8) + ((p_var1 * cal.dig_p2 as i64) << 12);
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

    let mut sck = Output::new(peripherals.PIN_10, Level::Low);
    let mut mosi = Output::new(peripherals.PIN_11, Level::Low);
    let miso = Input::new(peripherals.PIN_17, Pull::Up);
    let mut bmp_cs = Output::new(peripherals.PIN_16, Level::High);
    let _imu_cs = Output::new(peripherals.PIN_13, Level::High);

    let _ = writeln!(writer, "voloxide pico2w bmp280 bitbang probe");
    let _ = writeln!(
        writer,
        "sck=gp10 mosi=gp11 bmp_miso=gp17 bmp_cs=gp16 imu_cs=gp13 high"
    );

    let id = read_reg(&mut sck, &mut mosi, &miso, &mut bmp_cs, BMP280_ID_REG);
    let _ = writeln!(writer, "bmp280 chipid 0x{:02x}", id);
    if id != 0x58 {
        let _ = writeln!(
            writer,
            "unexpected chipid miso_idle={}",
            if miso.is_high() { "high" } else { "low" }
        );
        loop {
            delay_long();
        }
    }

    let cal = read_calibration(&mut sck, &mut mosi, &miso, &mut bmp_cs);
    let _ = writeln!(
        writer,
        "cal t1={} t2={} t3={} p1={}",
        cal.dig_t1, cal.dig_t2, cal.dig_t3, cal.dig_p1
    );

    write_reg(
        &mut sck,
        &mut mosi,
        &miso,
        &mut bmp_cs,
        BMP280_CONFIG_REG,
        0xa0,
    );
    write_reg(
        &mut sck,
        &mut mosi,
        &miso,
        &mut bmp_cs,
        BMP280_CTRL_MEAS_REG,
        0x4f,
    );

    loop {
        let (pressure, temperature) = read_sample(&mut sck, &mut mosi, &miso, &mut bmp_cs, &cal);
        let _ = writeln!(
            writer,
            "baro pressure={:.1} temp={:.2}",
            pressure, temperature
        );
        delay_long();
    }
}
