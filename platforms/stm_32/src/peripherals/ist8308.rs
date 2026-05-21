pub use embassy_stm32::mode::Async;
pub use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
pub use embassy_sync::signal::Signal;
use voloxide_core::{errors, packets};

// I2C Specific
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::i2c::I2c;
use embedded_hal_async::i2c::I2c as _;

// Polled Sensors
use crate::synch_at;
use embassy_time::Duration;
use embassy_time::Timer;

// Other

pub static MAG_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::MagPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::MagPacket, errors::SensorError>>::new();

pub struct Ist8308Sensor {
    pub dev: I2cDevice<
        'static,
        CriticalSectionRawMutex,
        I2c<'static, Async, embassy_stm32::i2c::mode::Master>,
    >,
}

impl Ist8308Sensor {
    async fn write_read(
        &mut self,
        address: u8,
        register: &[u8],
        data: &mut [u8],
    ) -> Result<(), ()> {
        match self.dev.write(address, register).await {
            Err(_e) => return Err(()),
            Ok(_) => {}
        }

        Timer::after(Duration::from_micros(0)).await;

        // Read register
        match self.dev.read(address, data).await {
            Err(_e) => return Err(()),
            Ok(_) => {}
        }

        Ok(())
    }

    pub async fn run(&mut self) {
        const ADDRESS: u8 = 0x0C;

        // Check device ID

        const WAI_REG: u8 = 0x00;
        let mut device_id = [0u8; 1];
        if self
            .write_read(ADDRESS, &[WAI_REG], &mut device_id)
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: reading WAI_REG",
            )));
            return;
        }
        const DEVICE_ID: u8 = 0x08;
        if device_id[0] != DEVICE_ID {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: bad device ID",
            )));
            return;
        }

        // Reset

        const CNTL3_REG: u8 = 0x32;
        const CNTL3_VAL_SRST: u8 = 1;
        if self
            .dev
            .write(ADDRESS, &[CNTL3_REG, CNTL3_VAL_SRST])
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing CNTL3_REG",
            )));
            return;
        }
        Timer::after(Duration::from_millis(20)).await; // allow 20 ms to reset

        //  Check status
        let mut cntrl3 = [0u8; 1];
        if self
            .write_read(ADDRESS, &[CNTL3_REG], &mut cntrl3)
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: reading CNTL3_REG",
            )));
            return;
        }
        if (cntrl3[0] & 0x01) != 0 {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: bad status CNTL3_REG",
            )));
            return;
        }

        // Configure

        // Enable DRDY (None Connected)
        const CNTL3_VAL_DRDY_EN: u8 = 1 << 3;

        if self
            .dev
            .write(ADDRESS, &[CNTL3_REG, CNTL3_VAL_DRDY_EN])
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing CNTL4_REG",
            )));
            return;
        }

        const CNTL4_REG: u8 = 0x34;
        const CNTL4_VAL_DYNAMIC_RANGE_500: u8 = 0;

        if self
            .dev
            .write(ADDRESS, &[CNTL4_REG, CNTL4_VAL_DYNAMIC_RANGE_500])
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing CNTL4_REG",
            )));
            return;
        }

        const OSRCNTL_REG: u8 = 0x41;
        const OSRCNTL_VAL_Y_16: u8 = 4 << 3;
        const OSRCNTL_VAL_XZ_16: u8 = 4;

        if self
            .dev
            .write(
                ADDRESS,
                &[OSRCNTL_REG, OSRCNTL_VAL_Y_16 | OSRCNTL_VAL_XZ_16],
            )
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing OSRCNTL_REG",
            )));
            return;
        }

        // Set ODR
        const CNTL2_REG: u8 = 0x31;
        const CNTL2_VAL_CONT_ODR100_MODE: u8 = 0x08; //Continuous (100Hz) mode
        if self
            .dev
            .write(ADDRESS, &[CNTL2_REG, CNTL2_VAL_CONT_ODR100_MODE])
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing CNTL2_REG",
            )));
            return;
        }

        let loop_period = Duration::from_hz(100);
        loop {
            let timestamp = synch_at(loop_period) + Duration::from_micros(900);
            Timer::at(timestamp).await;

            // Read Data
            const STAT1_REG: u8 = 0x10;
            let mut data = [0u8; 7];
            if self
                .write_read(ADDRESS, &[STAT1_REG], &mut data)
                .await
                .is_err()
            {
                MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                    "IST8308 Mag failed: reading STAT1_REG",
                )));
                continue;
            }

            const STAT1_VAL_DRDY: u8 = 0x01;
            let status = data[0];
            let data_ready = (status & STAT1_VAL_DRDY) != 0;
            if data_ready {
                let flux = [
                    f32::from((((data[2] as u16) << 8) | (data[1] as u16)) as i16) * 1.5e-7,
                    f32::from((((data[4] as u16) << 8) | (data[3] as u16)) as i16) * 1.5e-7,
                    f32::from((((data[6] as u16) << 8) | (data[5] as u16)) as i16) * 1.5e-7,
                ]; // Units of Tesla

                let timestamp_us = timestamp.as_micros();

                let header = packets::RosflightPacketHeader {
                    timestamp: timestamp_us,
                    status: status as u16,
                };

                let mag_packet = packets::MagPacket {
                    header,
                    flux,
                    temperature: 0.0f32,
                };
                MAG_SIGNAL.signal(Ok(mag_packet)); // make data available for other tasks
                // previous_timestamp_us = timestamp_us;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut ist: Ist8308Sensor) {
    ist.run().await;
}
