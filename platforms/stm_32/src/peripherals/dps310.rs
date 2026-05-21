use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Timer;
use embassy_time::with_timeout;
use embedded_hal_async::spi::SpiDevice as _;

use crate::synch_at;
use voloxide_core::errors;
use voloxide_core::packets;

// Device dependent
const SPI_READ: u8 = 0x80;
const SPI_WRITE: u8 = 0x00;

pub static BARO_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::BaroPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::BaroPacket, errors::SensorError>>::new();

pub struct Dps310Sensor {
    pub dev: SpiDevice<'static, CriticalSectionRawMutex, spi::Spi<'static, Async>, Output<'static>>,
    pub drdy: ExtiInput<'static>,
    pub three_wire: bool,
}

fn compliment(x: u32, bits: u32) -> f64 {
    let mut x = x as i32;
    if (x & (1i32 << (bits - 1))) != 0 {
        x -= 1i32 << bits;
    }
    f64::from(x)
}

const MEAS_CFG_REG: u8 = 0x08;
const ISR_REG: u8 = 0x0A;
const DPS310_READ_P_CMD: u8 = 0x00;
const DPS310_READ_T_CMD: u8 = 0x03;

const K1: f64 = 524288.0;
const K8: f64 = 7864320.0; //

impl Dps310Sensor {
    async fn read_register(&mut self, reg_addr: u8) -> Result<u8, errors::SensorError> {
        let tx = [reg_addr | SPI_READ, 0x00];
        let mut rx = [0u8; 2];
        self.dev.transfer(&mut rx, &tx).await.map_err(|e| match e {
            _ => errors::SensorError::GenericSensorError("SPI failed: read_register"),
        })?;
        Ok(rx[1])
    }

    async fn write_register(&mut self, reg_addr: u8, value: u8) -> Result<(), errors::SensorError> {
        let tx = [reg_addr | SPI_WRITE, value];
        // Soft Reset
        self.dev.write(&tx).await.map_err(|e| match e {
            _ => errors::SensorError::GenericSensorError("SPI failed: write_register"),
        })?;
        Ok(())
    }

    async fn initialize_sensor(&mut self) -> Result<[f64; 9], errors::SensorError> {
        // SOFT RESET
        const RESET_REG: u8 = 0x0C;
        self.write_register(RESET_REG, 0x09).await?;
        Timer::after_millis(52).await; // Wait reset (12ms) and for Coefficients to be ready (40ms).

        // 3-WIRE MODE & DRDY interrupts
        // Set to 3-wire or 4-wire SPI mode so we can read registers.
        // Interrupt and FIFO Config 0x09
        // 7 - 	1, DRDY active high
        // 6 - 	0, Disable FIFO full interrupt
        // 5 - 	1, Int on temp
        // 4 - 	1, Int on pressure
        // 3 - 	0, no Temp data shift
        // 2 - 	0, no Press data shift
        // 1 - 	0, Disable FIFO
        // 0 - 	1, 3-wire SPI interface
        const CFG_REG: u8 = 0x09;
        let three_wire_mode: u8 = if self.three_wire { 0x01 } else { 0x00 };
        self.write_register(CFG_REG, three_wire_mode | 0xB0).await?;

        // CHECK PRODUCT ID
        // there's a more concise way to do the if else, but I'm leaving it for now...
        const PRODUCT_ID_REG: u8 = 0x0D;
        const PRODUCT_ID: u8 = 0x10;
        let id = self.read_register(PRODUCT_ID_REG).await?;
        if id != PRODUCT_ID {
            //    "Failure: ID = {:#02x} failure. Should be {:#02x}",
            //    id,
            //    PRODUCT_ID
            //);
            return Err(errors::SensorError::GenericSensorError("ID mismatch"));
        }

        // CHECK IF CALIBRATION COEFFICIENTS ARE READY
        // again, better way to do if else is for future work
        const COEF_READY: u8 = 0x80;
        let coef_rdy = self.read_register(MEAS_CFG_REG).await?;
        if (coef_rdy & COEF_READY) == 0x00 {
            return Err(errors::SensorError::GenericSensorError(
                "Calibration coefficients not ready",
            ));
        }

        let cal = self.read_calibration_coefficients().await?;
        Ok(cal)
    }

    async fn read_calibration_coefficients(&mut self) -> Result<[f64; 9], errors::SensorError> {
        const COEF_REG: u8 = 0x10;
        let mut tx = [0u8; 19];
        tx[0] = COEF_REG | SPI_READ;

        let mut rx = [0u8; 19];
        self.dev.transfer(&mut rx, &tx).await.map_err(|e| match e {
            _ => {
                errors::SensorError::GenericSensorError("SPI failed: read_calibration_coefficients")
            }
        })?;

        // move u8 date into u32 data for bit manipulation
        let buf = rx.map(|x| x as u32);
        // compute coefficint values in f64
        let mut cal = [0f64; 9];
        cal[0] = compliment((buf[1] << 4) | ((buf[2] >> 4) & 0x0F), 12); // C0
        cal[1] = compliment(((buf[2] & 0x0F) << 8) | buf[3], 12); // C1
        cal[2] = compliment((buf[4] << 12) | (buf[5] << 4) | ((buf[6] >> 4) & 0x0F), 20); // C00
        cal[3] = compliment(((buf[6] & 0x0F) << 16) | (buf[7] << 8) | buf[8], 20); // C10
        cal[6] = compliment((buf[9] << 8) | buf[10], 16); // C01
        cal[7] = compliment((buf[11] << 8) | buf[12], 16); // C11
        cal[4] = compliment((buf[13] << 8) | buf[14], 16); // C20
        cal[8] = compliment((buf[15] << 8) | buf[16], 16); // C21
        cal[5] = compliment((buf[17] << 8) | buf[18], 16); // C30

        Ok(cal)
    }

    async fn pressure_config(&mut self) -> Result<(), errors::SensorError> {
        // PRESSURE CONFIG
        const PRS_CFG_REG: u8 = 0x06;
        self.write_register(PRS_CFG_REG, 0x03).await?; // 8x oversampling

        Ok(())
    }

    async fn temperature_config(&mut self) -> Result<(), errors::SensorError> {
        // CHECK TEMPERATURE SOURCE
        const COEF_SRCE_REG: u8 = 0x28;
        let temp_source = self.read_register(COEF_SRCE_REG).await? & 0x80;

        // TEMPERATURE CONFIG
        const TMP_CFG_REG: u8 = 0x07;
        self.write_register(TMP_CFG_REG, temp_source | 0x00).await?; //no oversampling

        Ok(())
    }

    async fn measurement_configuration(&mut self) -> Result<(), errors::SensorError> {
        // Measurement Configuration
        // 7 - 	0, read only
        // 6 - 	0, read only
        // 5 - 	0, read only
        // 4 - 	0, read only
        // 3 - 	0, reserved
        // 2:0 - 	111, pressure and temperature continuous mode
        // Set to idle
        self.write_register(MEAS_CFG_REG, 0x00).await?;

        Ok(())
    }

    async fn get_sensor_data(&mut self, cmd: u8) -> Result<i32, errors::SensorError> {
        let mut rx = [0u8; 4];
        self.dev
            .transfer(&mut rx, &[cmd | SPI_READ, 0, 0, 0])
            .await
            .map_err(|e| match e {
                _ => errors::SensorError::GenericSensorError("SPI failed: get_sensor_data"),
            })?;

        // Clear the ISR
        self.read_register(ISR_REG).await?;

        let raw = (((rx[1] as u32) << 24 | (rx[2] as u32) << 16 | (rx[3] as u32) << 8) as i32) >> 8;

        Ok(raw)
    }

    async fn get_pressure_data(&mut self) -> Result<(i32, u16), errors::SensorError> {
        // Start the Pressure read
        self.write_register(MEAS_CFG_REG, 0x01).await?;

        // wait for data ready...
        // Use DRDY signal for better robustness? otherwise, timeout at 14ms.
        let _drdy_result = with_timeout(
            Duration::from_micros(14_000),
            self.drdy.wait_for_rising_edge(),
        )
        .await
        .is_ok();
        Timer::after_micros(20).await; // We need at least 14us delay here if running at 2 MHz, maybe because of the messy harness?

        // read status (highest 8 bits)
        let status = (self.read_register(MEAS_CFG_REG).await? as u16) << 8;

        // read Pressure data
        let raw_p = self.get_sensor_data(DPS310_READ_P_CMD).await?;
        Ok((raw_p, status))
    }

    async fn get_temperature_data(&mut self) -> Result<(i32, u16), errors::SensorError> {
        // Start Temperature read
        self.write_register(MEAS_CFG_REG, 0x02).await?;

        // wait for data ready...
        // Use DRDY signal if available, otherwise let it timeout
        let _drdy_result = with_timeout(
            Duration::from_micros(3_000),
            self.drdy.wait_for_rising_edge(),
        )
        .await
        .is_ok();

        // read status (modify lowest 8 bits)
        let status_low = self.read_register(MEAS_CFG_REG).await? as u16;

        // read Temperature data
        let raw_t = self.get_sensor_data(DPS310_READ_T_CMD).await?;
        Ok((raw_t, status_low))
    }

    fn process_temperature_data(
        &mut self,
        raw_t: i32,
        raw_t_previous: &mut i32,
        cal: &[f64; 9],
    ) -> (f64, f64) {
        *raw_t_previous += (raw_t - *raw_t_previous) / 16; // filter temperature a bit (1/127 is cutoff frequenc of 100Hz * (1/16)/(2*pi) around 1 sec to 1/e)
        let raw_t_f64 = f64::from(*raw_t_previous) / K1;
        let temperature = cal[0] * 0.5 + cal[1] * raw_t_f64; // K

        (raw_t_f64, temperature)
    }

    fn process_pressure_data(&mut self, raw_p: i32, raw_t_f64: f64, cal: &[f64; 9]) -> (f64, f64) {
        let raw_p_f64 = f64::from(raw_p) / K8;
        let pressure = cal[2]
            + raw_p_f64 * (cal[3] + raw_p_f64 * (cal[4] + raw_p_f64 * cal[5]))
            + raw_t_f64 * (cal[6] + raw_p_f64 * (cal[7] + raw_p_f64 * cal[8])); // Pa
        (raw_p_f64, pressure)
    }

    pub async fn run(&mut self) {
        // initialize the sensor
        let mut cal = match self.initialize_sensor().await {
            Ok(cal) => cal,
            Err(e) => {
                BARO_SIGNAL.signal(Err(e));
                return;
            }
        };
        if let Err(e) = self.pressure_config().await {
            BARO_SIGNAL.signal(Err(e));
            return;
        }
        if let Err(e) = self.temperature_config().await {
            BARO_SIGNAL.signal(Err(e));
            return;
        }
        if let Err(e) = self.measurement_configuration().await {
            BARO_SIGNAL.signal(Err(e));
            return;
        }
        //////////////////////////////////////////////////////////////////////////////////////
        // Periodic Data Acquisition
        let mut raw_t_previous = 0_i32;
        let sample_period = Duration::from_hz(50);

        loop {
            let timestamp = synch_at(sample_period);
            Timer::at(timestamp).await;

            // process pressure data
            let (raw_p, status_high) = match self.get_pressure_data().await {
                Ok(data) => data,
                Err(e) => {
                    BARO_SIGNAL.signal(Err(e));
                    continue;
                }
            };
            let (raw_t, status_low) = match self.get_temperature_data().await {
                Ok(data) => data,
                Err(e) => {
                    BARO_SIGNAL.signal(Err(e));
                    continue;
                }
            };

            // combine status bits
            let status_combined = (status_high & 0xFF00) | (status_low & 0x00FF);

            let (raw_t_f64, temperature) =
                self.process_temperature_data(raw_t, &mut raw_t_previous, &mut cal);
            let (_raw_p_f64, pressure) = self.process_pressure_data(raw_p, raw_t_f64, &mut cal);

            if status_combined == 0xD0E0 {
                let header = packets::RosflightPacketHeader {
                    timestamp: timestamp.as_micros(),
                    status: status_combined,
                };
                let baro_packet = packets::BaroPacket {
                    header,
                    pressure: pressure as f32,
                    temperature: temperature as f32,
                    ..Default::default()
                };
                BARO_SIGNAL.signal(Ok(baro_packet)); // make data available for other tasks.
            } else {
                BARO_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("Bad status")));
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut dps: Dps310Sensor) {
    dps.run().await;
}
