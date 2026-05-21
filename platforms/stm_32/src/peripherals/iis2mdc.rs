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

//use defmt::info;
//use defmt::trace;

// Device dependent
const SPI_READ: u8 = 0x80;
const SPI_WRITE: u8 = 0x00;

// Registers
const OFFSET_REG: u8 = 0x45;
const WHO_AM_I_REG: u8 = 0x4F;

const CFG_REG_A: u8 = 0x60;
const CFG_REG_B: u8 = 0x61;
const CFG_REG_C: u8 = 0x62;
const INT_CTRL_REG: u8 = 0x63;
const INT_SOURCE_REG: u8 = 0x64;
const INT_THS_L_REG: u8 = 0x65;
const INT_THS_H_REG: u8 = 0x66;
const STATUS_REG: u8 = 0x67;
//const OUT_FLUX: u8 =  0x68;
const OUT_TEMP: u8 = 0x6E;

// Chip ID
const WHO_AM_I: u8 = 0x40;

pub static MAG_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::MagPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::MagPacket, errors::SensorError>>::new();

pub struct Iis2mdcSensor {
    pub dev: SpiDevice<'static, CriticalSectionRawMutex, spi::Spi<'static, Async>, Output<'static>>,
    pub drdy: ExtiInput<'static>,
}

impl Iis2mdcSensor {
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

    async fn initialize_sensor(&mut self) -> Result<u8, errors::SensorError> {
        // Check Chip ID
        let who_am_i = self.read_register(WHO_AM_I_REG).await?;
        if who_am_i == WHO_AM_I {
            //info!("WHO_AM_I = {:#02x} success", who_am_i);
        } else {
            //info!("WHO_AM_I = {:#02x} failure", who_am_i);
            return Err(errors::SensorError::GenericSensorError(
                "IIS2MDC Chip ID mismatch",
            ));
        }

        // Reset the sensor:
        // Register A (0x60)
        // 7:  = 0 COMP_TEMP_EN Temp comp enable
        // 6:  = X REBOOT
        // 5:  = X SOFT_RST
        // 4:  = 0 High resolution Mode (LP=0)
        // 3:2 = 00 10 Hz Data Rate (ODR)
        // 1:0 = 00 for continuous Mode,  10 or 11 = idle mode
        self.write_register(CFG_REG_A, 0x20).await?;
        Timer::after_micros(10).await; // Wait at least 5 us after reset
        self.write_register(CFG_REG_A, 0x40).await?;
        Timer::after_micros(21).await; // wait at least 20 ms for reboot to complete

        // Configure sensor:
        // Set to idle (awaiting read command)
        // Register A (0x60)
        // 7:  = 1 COMP_TEMP_EN Temp comp enable
        // 6:  = 0 REBOOT
        // 5:  = 0 SOFT_RST
        // 4:  = 0 High resolution Mode (High resolution = 0, Low power =1)
        //
        // 3:2 = 11 = 100 Hz Data Rate (ODR)
        // 1:0 = 00 Continuous Mode, 01 = single mode, 11 or 10 is idle mode.
        //	write_register(0x60,0x81); // 1000 0001 = 0x81 For Single Acq
        // 1000 1100 = 0x8C For 100 Hz.
        // writeRegister(CFG_REG_A,0x80|(odr_mode<<2)); // continuous mode

        self.write_register(CFG_REG_A, 0x83).await?;

        // Set Single Measurement Mode
        // Register B (0x61)
        // [7:5] 000
        // [4]  1 OFF_CANC_ONE_SHOT 1=Offset Cancellation in single mode
        // [3]  0
        // [2]  0 Set Freq of Set pulse to 63 ODR
        // [1]  1, OFF_CANC 1= enable offset cancellation in single mode
        // [0]  0 LPF disable offset filter (1- enabled)

        self.write_register(CFG_REG_B, 0x12).await?;

        // Enable DRDY Pin and set SPI only mode (no I2C)
        // Register C (0x62)
        // 7: =0 Unused
        // 6: =0 INT_on_PIN Enable event interrupts
        // 5: =1 I2C_DIS (Disable I2C interface use only SPI)
        // 4: =1 BDU
        //
        // 3: =0 BLE do not swap data bytes
        // 2: =0 Unused
        // 1: =0 SELF_TEST
        // 0: =1 DRDY_on_PIN Enable DRDY

        self.write_register(CFG_REG_C, 0x31).await?;

        // Disable Event Interrupts (this is not DRDY)

        self.write_register(INT_CTRL_REG, 0x00).await?;
        self.write_register(INT_SOURCE_REG, 0x00).await?;
        self.write_register(INT_THS_L_REG, 0x00).await?;
        self.write_register(INT_THS_H_REG, 0x00).await?;

        // Read the status register
        let status = self.read_register(STATUS_REG).await?;
        //info!("STATUS_SENSOR = {:#02x}. Shold be 0x00", status);

        // Read calibration registers
        let mut offset = [0u8; 7];
        self.dev
            .transfer(&mut offset, &[OFFSET_REG | SPI_READ, 0, 0, 0, 0, 0, 0])
            .await
            .map_err(|e| match e {
                _ => errors::SensorError::GenericSensorError("SPI failed: initialize_sensor"),
            })?;

        let _offset = [
            (((offset[1] as u16) | (offset[2] as u16) << 8) as i16) * 3 / 2,
            (((offset[3] as u16) | (offset[4] as u16) << 8) as i16) * 3 / 2,
            (((offset[5] as u16) | (offset[6] as u16) << 8) as i16) * 3 / 2,
        ];

        //info!("OFFSET_REGISTERS (not used) = {:?}", offset);

        Ok(status)
    }

    async fn get_flux_and_temperature_data(
        &mut self,
    ) -> Result<(u16, [u8; 8], [u8; 3]), errors::SensorError> {
        // Start Read Cycle
        self.write_register(CFG_REG_A, 0x81).await?;

        // Use DRDY signal for better robustness? otherwise, timeout at 9.6ms.
        let _drdy_result = with_timeout(
            Duration::from_micros(9_600),
            self.drdy.wait_for_rising_edge(),
        )
        .await
        .is_ok();

        let mut flux_data = [0u8; 8];
        // note starting at STATUS_REG to capture both status and flux values
        self.dev
            .transfer(
                &mut flux_data,
                &[STATUS_REG | SPI_READ, 0, 0, 0, 0, 0, 0, 0],
            )
            .await
            .map_err(|e| match e {
                _ => errors::SensorError::GenericSensorError(
                    "SPI failed: get_flux_and_temperature_data",
                ),
            })?;

        let mut raw_temperature = [0u8; 3];
        self.dev
            .transfer(&mut raw_temperature, &[OUT_TEMP | SPI_READ, 0, 0])
            .await
            .map_err(|e| match e {
                _ => errors::SensorError::GenericSensorError(
                    "SPI failed: get_flux_and_temperature_data",
                ),
            })?;

        let status = flux_data[1] as u16;
        Ok((status, flux_data, raw_temperature))
    }

    fn process_flux_data(&self, flux_data: [u8; 8], flux_previous: &mut [i16; 3]) -> [f32; 3] {
        let raw_flux = [
            ((flux_data[2] as u16) | ((flux_data[3] as u16) << 8)) as i16,
            ((flux_data[4] as u16) | ((flux_data[5] as u16) << 8)) as i16,
            ((flux_data[6] as u16) | ((flux_data[7] as u16) << 8)) as i16,
        ];

        let flux = [
            -f32::from(raw_flux[0] + flux_previous[0]) / 2. * 1.5e-7, // convert to Tesla
            f32::from(raw_flux[1] + flux_previous[1]) / 2. * 1.5e-7,
            f32::from(raw_flux[2] + flux_previous[2]) / 2. * 1.5e-7,
        ];

        *flux_previous = raw_flux;

        flux
    }

    fn process_temperature_data(&self, raw_temperature: [u8; 3]) -> f32 {
        let temperature =
            f32::from(((raw_temperature[1] as u16) | ((raw_temperature[2] as u16) << 8)) as i16)
                / 8.0
                + 25.0;
        temperature
    }

    pub async fn run(&mut self) {
        let _status = match self.initialize_sensor().await {
            Ok(status) => status,
            Err(e) => {
                //trace!("Failed to initialize sensor: {:?}", e);
                MAG_SIGNAL.signal(Err(e));
                return;
            }
        };

        let mut flux_previous = [0_i16; 3];
        let sample_period = Duration::from_hz(100);

        loop {
            let timestamp = synch_at(sample_period);
            Timer::at(timestamp).await;

            // Read Flux Data
            let (status, flux_data, raw_temperature) =
                match self.get_flux_and_temperature_data().await {
                    Ok(data) => data,
                    Err(e) => {
                        MAG_SIGNAL.signal(Err(e));
                        continue;
                    }
                };

            if status == 0x000F {
                let flux = self.process_flux_data(flux_data, &mut flux_previous);
                let temperature = self.process_temperature_data(raw_temperature);

                let header = packets::RosflightPacketHeader {
                    timestamp: timestamp.as_micros(),
                    status,
                };
                let mag_packet = packets::MagPacket {
                    header,
                    flux,
                    temperature,
                };
                MAG_SIGNAL.signal(Ok(mag_packet)); // make data available for other tasks
            } else {
                MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("Bad Status")));
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut iis: Iis2mdcSensor) {
    iis.run().await;
}
