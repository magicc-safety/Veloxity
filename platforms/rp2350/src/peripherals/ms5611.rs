use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiDevice;
use veloxity_core::{
    errors::SensorError,
    packets::{BaroPacket, RosflightPacketHeader},
};

const CMD_RESET: u8 = 0x1e;
const CMD_ADC_READ: u8 = 0x00;
const CMD_CONVERT_D1_OSR4096: u8 = 0x48;
const CMD_CONVERT_D2_OSR4096: u8 = 0x58;
const CMD_PROM_READ_BASE: u8 = 0xa0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ms5611Error {
    Spi,
    InvalidProm,
}

impl Ms5611Error {
    pub fn sensor_error(self) -> SensorError {
        match self {
            Ms5611Error::Spi => SensorError::GenericSensorError("ms5611 spi error"),
            Ms5611Error::InvalidProm => SensorError::GenericSensorError("ms5611 invalid prom"),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ms5611Calibration {
    pub pressure_sens: u16,
    pub pressure_offset: u16,
    pub temp_coeff_pressure_sens: u16,
    pub temp_coeff_pressure_offset: u16,
    pub reference_temp: u16,
    pub temp_coeff_temp: u16,
}

pub struct Ms5611<SPI> {
    spi: SPI,
    calibration: Ms5611Calibration,
    initialized: bool,
    last_sample_us: u64,
}

impl<SPI> Ms5611<SPI>
where
    SPI: SpiDevice,
{
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            calibration: Ms5611Calibration::default(),
            initialized: false,
            last_sample_us: 0,
        }
    }

    pub async fn init(&mut self) -> Result<(), Ms5611Error> {
        self.spi
            .write(&[CMD_RESET])
            .await
            .map_err(|_| Ms5611Error::Spi)?;
        Timer::after_millis(3).await;

        let mut prom = [0_u16; 8];
        for (index, word) in prom.iter_mut().enumerate() {
            *word = self.read_prom_word(index as u8).await?;
        }

        if prom[1..7].iter().all(|word| *word == 0) {
            return Err(Ms5611Error::InvalidProm);
        }

        self.calibration = Ms5611Calibration {
            pressure_sens: prom[1],
            pressure_offset: prom[2],
            temp_coeff_pressure_sens: prom[3],
            temp_coeff_pressure_offset: prom[4],
            reference_temp: prom[5],
            temp_coeff_temp: prom[6],
        };
        self.initialized = true;
        Ok(())
    }

    pub async fn sample_baro(
        &mut self,
        now_us: u64,
        min_interval_us: u64,
    ) -> Result<Option<BaroPacket>, Ms5611Error> {
        if !self.initialized {
            self.init().await?;
        }
        if now_us < self.last_sample_us.saturating_add(min_interval_us) {
            return Ok(None);
        }

        let pressure_raw = self.convert_and_read(CMD_CONVERT_D1_OSR4096).await?;
        let temp_raw = self.convert_and_read(CMD_CONVERT_D2_OSR4096).await?;
        let (pressure, temperature) = compensate(&self.calibration, pressure_raw, temp_raw);
        self.last_sample_us = now_us;

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

    async fn read_prom_word(&mut self, index: u8) -> Result<u16, Ms5611Error> {
        let tx = [CMD_PROM_READ_BASE + index * 2, 0, 0];
        let mut rx = [0_u8; 3];
        self.spi
            .transfer(&mut rx, &tx)
            .await
            .map_err(|_| Ms5611Error::Spi)?;
        Ok(u16::from_be_bytes([rx[1], rx[2]]))
    }

    async fn convert_and_read(&mut self, command: u8) -> Result<u32, Ms5611Error> {
        self.spi
            .write(&[command])
            .await
            .map_err(|_| Ms5611Error::Spi)?;
        Timer::after(Duration::from_micros(9_100)).await;
        let tx = [CMD_ADC_READ, 0, 0, 0];
        let mut rx = [0_u8; 4];
        self.spi
            .transfer(&mut rx, &tx)
            .await
            .map_err(|_| Ms5611Error::Spi)?;
        Ok(((rx[1] as u32) << 16) | ((rx[2] as u32) << 8) | rx[3] as u32)
    }
}

fn compensate(cal: &Ms5611Calibration, d1: u32, d2: u32) -> (f32, f32) {
    let d_t = d2 as i64 - ((cal.reference_temp as i64) << 8);
    let temp = 2000 + d_t * cal.temp_coeff_temp as i64 / 8_388_608;
    let off =
        ((cal.pressure_offset as i64) << 16) + (cal.temp_coeff_pressure_offset as i64 * d_t) / 128;
    let sens =
        ((cal.pressure_sens as i64) << 15) + (cal.temp_coeff_pressure_sens as i64 * d_t) / 256;

    let pressure_pa = (((d1 as i64 * sens) / 2_097_152 - off) / 32_768) as f32;
    let temperature_c = temp as f32 / 100.0;
    (pressure_pa, temperature_c)
}
