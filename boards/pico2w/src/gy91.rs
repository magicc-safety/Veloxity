use rp2350_platform::hal::{
    gpio::Output,
    peripherals::SPI1,
    spi::{Blocking, Spi},
};
use veloxity_core::{
    errors::SensorError,
    packets::{BaroPacket, ImuPacket, RosflightPacketHeader},
};

const GRAVITY: f32 = 9.80665;
const DEG_TO_RAD: f32 = 0.017453292519943295;

const MPU_WHO_AM_I: u8 = 0x75;
const MPU_PWR_MGMT_1: u8 = 0x6b;
const MPU_SMPLRT_DIV: u8 = 0x19;
const MPU_CONFIG: u8 = 0x1a;
const MPU_GYRO_CONFIG: u8 = 0x1b;
const MPU_ACCEL_CONFIG: u8 = 0x1c;
const MPU_ACCEL_CONFIG2: u8 = 0x1d;
const MPU_ACCEL_XOUT_H: u8 = 0x3b;

const BMP_CHIP_ID: u8 = 0xd0;
const BMP_CALIB_START: u8 = 0x88;
const BMP_CTRL_MEAS: u8 = 0xf4;
const BMP_CONFIG: u8 = 0xf5;
const BMP_PRESS_MSB: u8 = 0xf7;

#[cfg(feature = "imu-400hz")]
pub const GY91_IMU_SAMPLE_INTERVAL_US: u64 = 2_500;
#[cfg(not(feature = "imu-400hz"))]
pub const GY91_IMU_SAMPLE_INTERVAL_US: u64 = 2_000;
pub const GY91_IMU_SAMPLE_RATE_HZ: u32 = 1_000_000 / GY91_IMU_SAMPLE_INTERVAL_US as u32;
pub const GY91_BARO_SAMPLE_INTERVAL_US: u64 = 20_000;
pub const GY91_BARO_SAMPLE_RATE_HZ: u32 = 1_000_000 / GY91_BARO_SAMPLE_INTERVAL_US as u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gy91Error {
    Spi,
    InvalidMpuId(u8),
    InvalidBmpId(u8),
}

impl Gy91Error {
    pub fn sensor_error(self) -> SensorError {
        match self {
            Gy91Error::Spi => SensorError::GenericSensorError("gy91 spi error"),
            Gy91Error::InvalidMpuId(_) => SensorError::GenericSensorError("gy91 invalid mpu id"),
            Gy91Error::InvalidBmpId(_) => SensorError::GenericSensorError("gy91 invalid bmp id"),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Gy91Ids {
    pub mpu: u8,
    pub bmp: u8,
}

#[derive(Default)]
struct Bmp280Calibration {
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

pub struct Gy91 {
    spi: Spi<'static, SPI1, Blocking>,
    mpu_cs: Output<'static>,
    bmp_cs: Output<'static>,
    bmp_calibration: Bmp280Calibration,
    initialized: bool,
    bmp_initialized: bool,
    bmp_id: Option<u8>,
    ids: Option<Gy91Ids>,
    imu_seq: u32,
    last_imu_sample_us: u64,
    last_baro_sample_us: u64,
}

impl Gy91 {
    pub fn new(
        spi: Spi<'static, SPI1, Blocking>,
        mut mpu_cs: Output<'static>,
        mut bmp_cs: Output<'static>,
    ) -> Self {
        mpu_cs.set_high();
        bmp_cs.set_high();
        Self {
            spi,
            mpu_cs,
            bmp_cs,
            bmp_calibration: Bmp280Calibration::default(),
            initialized: false,
            bmp_initialized: false,
            bmp_id: None,
            ids: None,
            imu_seq: 0,
            last_imu_sample_us: 0,
            last_baro_sample_us: 0,
        }
    }

    pub fn ids(&self) -> Option<Gy91Ids> {
        self.ids
    }

    pub fn init(&mut self) -> Result<Gy91Ids, Gy91Error> {
        let mpu = self.mpu_read_reg(MPU_WHO_AM_I)?;
        if !matches!(mpu, 0x70 | 0x71 | 0x73) {
            return Err(Gy91Error::InvalidMpuId(mpu));
        }

        let bmp = self.init_bmp280()?;

        self.mpu_write_reg(MPU_PWR_MGMT_1, 0x01)?;
        self.mpu_write_reg(MPU_CONFIG, 0x03)?;
        self.mpu_write_reg(MPU_SMPLRT_DIV, 0x00)?;
        self.mpu_write_reg(MPU_GYRO_CONFIG, 0x08)?;
        self.mpu_write_reg(MPU_ACCEL_CONFIG, 0x08)?;
        self.mpu_write_reg(MPU_ACCEL_CONFIG2, 0x03)?;

        let ids = Gy91Ids { mpu, bmp };
        self.initialized = true;
        self.ids = Some(ids);
        Ok(ids)
    }

    pub fn sample_imu(&mut self, now_us: u64) -> Result<Option<ImuPacket<f32>>, Gy91Error> {
        self.ensure_initialized()?;
        if now_us
            < self
                .last_imu_sample_us
                .saturating_add(GY91_IMU_SAMPLE_INTERVAL_US)
        {
            return Ok(None);
        }

        let packet = self.sample_imu_unthrottled(now_us)?;
        self.last_imu_sample_us = now_us;
        Ok(Some(packet))
    }

    pub fn sample_imu_unthrottled(&mut self, now_us: u64) -> Result<ImuPacket<f32>, Gy91Error> {
        self.ensure_initialized()?;
        let mut raw = [0_u8; 14];
        self.mpu_read_regs(MPU_ACCEL_XOUT_H, &mut raw)?;

        let accel_x = i16_be(raw[0], raw[1]) as f32;
        let accel_y = i16_be(raw[2], raw[3]) as f32;
        let accel_z = i16_be(raw[4], raw[5]) as f32;
        let temp_raw = i16_be(raw[6], raw[7]) as f32;
        let gyro_x = i16_be(raw[8], raw[9]) as f32;
        let gyro_y = i16_be(raw[10], raw[11]) as f32;
        let gyro_z = i16_be(raw[12], raw[13]) as f32;

        self.imu_seq = self.imu_seq.wrapping_add(1);

        Ok(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: now_us,
                status: 0,
            },
            accel: [
                accel_x / 8192.0 * GRAVITY,
                accel_y / 8192.0 * GRAVITY,
                accel_z / 8192.0 * GRAVITY,
            ],
            gyro: [
                gyro_x / 65.5 * DEG_TO_RAD,
                gyro_y / 65.5 * DEG_TO_RAD,
                gyro_z / 65.5 * DEG_TO_RAD,
            ],
            temperature: temp_raw / 333.87 + 21.0,
            seq: self.imu_seq,
        })
    }

    pub fn sample_baro(&mut self, now_us: u64) -> Result<Option<BaroPacket>, Gy91Error> {
        self.ensure_baro_initialized()?;
        if now_us
            < self
                .last_baro_sample_us
                .saturating_add(GY91_BARO_SAMPLE_INTERVAL_US)
        {
            return Ok(None);
        }

        let mut raw = [0_u8; 6];
        self.bmp_read_regs(BMP_PRESS_MSB, &mut raw)?;
        let adc_p =
            (((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4)) as i32;
        let adc_t =
            (((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4)) as i32;

        let (pressure, temperature) = compensate_bmp280(&self.bmp_calibration, adc_p, adc_t);
        self.last_baro_sample_us = now_us;

        Ok(Some(BaroPacket {
            header: RosflightPacketHeader {
                timestamp: now_us,
                status: 0,
            },
            pressure,
            temperature,
            altitude: 0.0,
        }))
    }

    fn ensure_initialized(&mut self) -> Result<(), Gy91Error> {
        if self.initialized {
            Ok(())
        } else {
            self.init().map(|_| ())
        }
    }

    fn ensure_baro_initialized(&mut self) -> Result<(), Gy91Error> {
        if self.bmp_initialized {
            Ok(())
        } else {
            self.init_bmp280().map(|_| ())
        }
    }

    fn init_bmp280(&mut self) -> Result<u8, Gy91Error> {
        if self.bmp_initialized {
            return self.bmp_id.ok_or(Gy91Error::Spi);
        }

        let bmp = self.bmp_read_reg(BMP_CHIP_ID)?;
        if bmp != 0x58 {
            return Err(Gy91Error::InvalidBmpId(bmp));
        }

        self.bmp_calibration = self.read_bmp_calibration()?;
        self.bmp_write_reg(BMP_CONFIG, 0xa0)?;
        self.bmp_write_reg(BMP_CTRL_MEAS, 0x4f)?;
        self.bmp_initialized = true;
        self.bmp_id = Some(bmp);
        Ok(bmp)
    }

    fn mpu_read_reg(&mut self, reg: u8) -> Result<u8, Gy91Error> {
        let mut out = [0_u8; 1];
        self.mpu_read_regs(reg, &mut out)?;
        Ok(out[0])
    }

    fn mpu_read_regs(&mut self, reg: u8, out: &mut [u8]) -> Result<(), Gy91Error> {
        let mut txrx = [0_u8; 16];
        if out.len() + 1 > txrx.len() {
            return Err(Gy91Error::Spi);
        }
        txrx[0] = reg | 0x80;
        self.mpu_cs.set_low();
        let result = self
            .spi
            .blocking_transfer_in_place(&mut txrx[..out.len() + 1]);
        self.mpu_cs.set_high();
        result.map_err(|_| Gy91Error::Spi)?;
        out.copy_from_slice(&txrx[1..out.len() + 1]);
        Ok(())
    }

    fn mpu_write_reg(&mut self, reg: u8, value: u8) -> Result<(), Gy91Error> {
        let mut bytes = [reg & 0x7f, value];
        self.mpu_cs.set_low();
        let result = self.spi.blocking_transfer_in_place(&mut bytes);
        self.mpu_cs.set_high();
        result.map_err(|_| Gy91Error::Spi)
    }

    fn bmp_read_reg(&mut self, reg: u8) -> Result<u8, Gy91Error> {
        let mut out = [0_u8; 1];
        self.bmp_read_regs(reg, &mut out)?;
        Ok(out[0])
    }

    fn bmp_read_regs(&mut self, reg: u8, out: &mut [u8]) -> Result<(), Gy91Error> {
        let mut txrx = [0_u8; 32];
        if out.len() + 1 > txrx.len() {
            return Err(Gy91Error::Spi);
        }
        txrx[0] = reg | 0x80;
        self.bmp_cs.set_low();
        let result = self
            .spi
            .blocking_transfer_in_place(&mut txrx[..out.len() + 1]);
        self.bmp_cs.set_high();
        result.map_err(|_| Gy91Error::Spi)?;
        out.copy_from_slice(&txrx[1..out.len() + 1]);
        Ok(())
    }

    fn bmp_write_reg(&mut self, reg: u8, value: u8) -> Result<(), Gy91Error> {
        let mut bytes = [reg & 0x7f, value];
        self.bmp_cs.set_low();
        let result = self.spi.blocking_transfer_in_place(&mut bytes);
        self.bmp_cs.set_high();
        result.map_err(|_| Gy91Error::Spi)
    }

    fn read_bmp_calibration(&mut self) -> Result<Bmp280Calibration, Gy91Error> {
        let mut raw = [0_u8; 24];
        self.bmp_read_regs(BMP_CALIB_START, &mut raw)?;
        Ok(Bmp280Calibration {
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
}

fn i16_be(msb: u8, lsb: u8) -> i16 {
    i16::from_be_bytes([msb, lsb])
}

fn u16_le(lsb: u8, msb: u8) -> u16 {
    u16::from_le_bytes([lsb, msb])
}

fn i16_le(lsb: u8, msb: u8) -> i16 {
    i16::from_le_bytes([lsb, msb])
}

fn compensate_bmp280(cal: &Bmp280Calibration, adc_p: i32, adc_t: i32) -> (f32, f32) {
    let var1 = (((adc_t >> 3) - ((cal.dig_t1 as i32) << 1)) * cal.dig_t2 as i32) >> 11;
    let var2 = (((((adc_t >> 4) - cal.dig_t1 as i32) * ((adc_t >> 4) - cal.dig_t1 as i32)) >> 12)
        * cal.dig_t3 as i32)
        >> 14;
    let t_fine = var1 + var2;
    let temperature = ((t_fine * 5 + 128) >> 8) as f32 / 100.0;

    let mut var1 = t_fine as i64 - 128_000;
    let mut var2 = var1 * var1 * cal.dig_p6 as i64;
    var2 += (var1 * cal.dig_p5 as i64) << 17;
    var2 += (cal.dig_p4 as i64) << 35;
    var1 = ((var1 * var1 * cal.dig_p3 as i64) >> 8) + ((var1 * cal.dig_p2 as i64) << 12);
    var1 = (((1_i64 << 47) + var1) * cal.dig_p1 as i64) >> 33;
    if var1 == 0 {
        return (0.0, temperature);
    }

    let mut p = 1_048_576_i64 - adc_p as i64;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (cal.dig_p9 as i64 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (cal.dig_p8 as i64 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((cal.dig_p7 as i64) << 4);

    (p as f32 / 256.0, temperature)
}
