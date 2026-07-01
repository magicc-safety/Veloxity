use veloxity_core::board::BoardIo;
use veloxity_core::errors;
use veloxity_core::math::FlightFloat;
use veloxity_core::params::Params;
use veloxity_core::sensors::SensorBus;

use embassy_time::Delay;
use stm_32::cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
#[cfg(not(feature = "scope-timing-pins"))]
use stm_32::cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use stm_32::peripherals;
use stm_32::peripherals::pwm::PixRacerProServoMonstrosity;
use stm_32::*;

include!("../../../platforms/stm_32/stm32h7x3_common.rs");

static mut PARAM_STORE: Option<Params> = None;

#[cfg(feature = "sensor-poll-diagnostics")]
mod sensor_poll_diagnostics {
    use core::sync::atomic::{AtomicU32, Ordering};

    use veloxity_core::{errors::SensorError, math::FlightFloat};

    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_IMU: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_IMU_ERR: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_MAG: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_MAG_ERR: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_BARO: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_BARO_ERR: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_PITOT: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_PITOT_ERR: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_RANGE: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_RANGE_ERR: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_GNSS: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_GNSS_ERR: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_RC: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static VELOXITY_PIXRACER_DIAG_RC_ERR: AtomicU32 = AtomicU32::new(0);

    pub fn record_bus<R: FlightFloat>(sensors: &veloxity_core::sensors::SensorBus<R>) {
        record(
            &sensors.imu,
            &VELOXITY_PIXRACER_DIAG_IMU,
            &VELOXITY_PIXRACER_DIAG_IMU_ERR,
        );
        record(
            &sensors.mag,
            &VELOXITY_PIXRACER_DIAG_MAG,
            &VELOXITY_PIXRACER_DIAG_MAG_ERR,
        );
        record(
            &sensors.baro,
            &VELOXITY_PIXRACER_DIAG_BARO,
            &VELOXITY_PIXRACER_DIAG_BARO_ERR,
        );
        record(
            &sensors.pitot,
            &VELOXITY_PIXRACER_DIAG_PITOT,
            &VELOXITY_PIXRACER_DIAG_PITOT_ERR,
        );
        record(
            &sensors.range,
            &VELOXITY_PIXRACER_DIAG_RANGE,
            &VELOXITY_PIXRACER_DIAG_RANGE_ERR,
        );
        record(
            &sensors.gnss,
            &VELOXITY_PIXRACER_DIAG_GNSS,
            &VELOXITY_PIXRACER_DIAG_GNSS_ERR,
        );
        record(
            &sensors.rc,
            &VELOXITY_PIXRACER_DIAG_RC,
            &VELOXITY_PIXRACER_DIAG_RC_ERR,
        );
    }

    fn record<T>(sample: &Option<Result<T, SensorError>>, count: &AtomicU32, errors: &AtomicU32) {
        if let Some(result) = sample {
            count.fetch_add(1, Ordering::Relaxed);
            if result.is_err() {
                errors.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

fn spawn_task<S: Send>(
    spawner: &embassy_executor::SendSpawner,
    token: Result<embassy_executor::SpawnToken<S>, embassy_executor::SpawnError>,
) {
    spawner.spawn(token.expect("failed to allocate Embassy task"));
}

pub struct Board {
    _probe: [Output<'static>; 3],
    pub start_time: embassy_time::Instant,
    test_pin_1: Output<'static>,
    test_pin_2: Output<'static>,
    pending_reset_to_bootloader: Option<bool>,
    #[cfg(feature = "sensor-poll-diagnostics")]
    last_sbus_diag_ms: u32,
    #[cfg(feature = "sensor-poll-diagnostics")]
    sbus_rc_drains: u32,
    #[cfg(feature = "sensor-poll-diagnostics")]
    sbus_last_rc_status: u32,
    #[cfg(feature = "sensor-poll-diagnostics")]
    sbus_last_rc_lol: bool,
}

impl BoardIo for Board {
    fn set_test_pin_1(&mut self, high: bool) {
        if high {
            self.test_pin_1.set_high();
        } else {
            self.test_pin_1.set_low();
        }
    }

    fn set_test_pin_2(&mut self, high: bool) {
        if high {
            self.test_pin_2.set_high();
        } else {
            self.test_pin_2.set_low();
        }
    }

    fn update_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        sensors.imu = peripherals::bmi08x::IMU_SIGNAL
            .try_take()
            .map(|result| result.map(|packet| packet.cast()));
        sensors.mag = peripherals::ist8308::MAG_SIGNAL.try_take();
        sensors.baro = peripherals::dps310::BARO_SIGNAL.try_take();
        sensors.pitot = peripherals::ms4525::PITOT_SIGNAL.try_take();
        sensors.range = peripherals::llv3hp::RANGE_SIGNAL.try_take();
        sensors.gnss = peripherals::ublox::GNSS_SIGNAL.try_take();
        sensors.rc = peripherals::sbus::RC_SIGNAL.try_take();
        #[cfg(feature = "sensor-poll-diagnostics")]
        self.record_sbus_rc_drain(&sensors.rc);
        #[cfg(feature = "sensor-poll-diagnostics")]
        sensor_poll_diagnostics::record_bus(sensors);
        #[cfg(feature = "sensor-poll-diagnostics")]
        self.log_sbus_diagnostics_if_due();
        #[cfg(not(feature = "scope-timing-pins"))]
        if sensors.imu.is_some() {
            let mut delay = Delay;
            self.set_test_pin_1(true);
            delay.delay_us(1u32);
            self.set_test_pin_1(false);
        }
    }

    fn imu_pending(&self) -> bool {
        peripherals::bmi08x::IMU_SIGNAL.signaled()
    }

    fn update_imu_sensor<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        sensors.imu = peripherals::bmi08x::IMU_SIGNAL
            .try_take()
            .map(|result| result.map(|packet| packet.cast()));
        #[cfg(feature = "sensor-poll-diagnostics")]
        sensor_poll_diagnostics::record_bus(sensors);
        #[cfg(not(feature = "scope-timing-pins"))]
        if sensors.imu.is_some() {
            let mut delay = Delay;
            self.set_test_pin_1(true);
            delay.delay_us(1u32);
            self.set_test_pin_1(false);
        }
    }

    fn update_service_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        sensors.mag = peripherals::ist8308::MAG_SIGNAL.try_take();
        sensors.baro = peripherals::dps310::BARO_SIGNAL.try_take();
        sensors.pitot = peripherals::ms4525::PITOT_SIGNAL.try_take();
        sensors.range = peripherals::llv3hp::RANGE_SIGNAL.try_take();
        sensors.gnss = peripherals::ublox::GNSS_SIGNAL.try_take();
        sensors.rc = peripherals::sbus::RC_SIGNAL.try_take();
        #[cfg(feature = "sensor-poll-diagnostics")]
        self.record_sbus_rc_drain(&sensors.rc);
        #[cfg(feature = "sensor-poll-diagnostics")]
        sensor_poll_diagnostics::record_bus(sensors);
        #[cfg(feature = "sensor-poll-diagnostics")]
        self.log_sbus_diagnostics_if_due();
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        #[cfg(feature = "usb-vcp-serial")]
        match peripherals::vcp::VCP_RX.try_read(buf) {
            Ok(n) => {
                return Some(Ok(n));
            }
            Err(embassy_sync::pipe::TryReadError::Empty) => {
                // This is NORMAL. Do not log an error.
                // Return Ok(0) to indicate "no bytes right now" without erroring.
                return Some(Ok(0));
            }
        }

        #[cfg(not(feature = "usb-vcp-serial"))]
        match peripherals::telem::TELEM_RX.try_read(buf) {
            Ok(n) => {
                return Some(Ok(n));
            }
            Err(embassy_sync::pipe::TryReadError::Empty) => {
                // This is NORMAL. Do not log an error.
                // Return Ok(0) to indicate "no bytes right now" without erroring.
                return Some(Ok(0));
            }
        }
    }
    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        #[cfg(feature = "usb-vcp-serial")]
        {
            let mut n = 0;
            let len = bytes.len();
            loop {
                match peripherals::vcp::VCP_TX.try_write(&bytes[n..len]) {
                    Ok(wrote) => {
                        if wrote == (len - n) {
                            break;
                        } else {
                            n += wrote;
                        }
                    }
                    Err(_) => {
                        return Some(Err(errors::TelemError::GenericTelemError(
                            "Error Writing USB VCP Packet!",
                        )));
                    }
                }
            }
            return Some(Ok(len));
        }

        #[cfg(not(feature = "usb-vcp-serial"))]
        {
            let mut n = 0;
            let len = bytes.len();
            loop {
                match peripherals::telem::TELEM_TX.try_write(&bytes[n..len]) {
                    Ok(wrote) => {
                        if wrote == (len - n) {
                            break;
                        } else {
                            n += wrote;
                        }
                    }
                    Err(_) => {
                        return Some(Err(errors::TelemError::GenericTelemError(
                            "Error Writing Telem Packet!",
                        )));
                    }
                }
            }
            Some(Ok(len))
        }
    }

    fn clock_millis(&self) -> u32 {
        self.start_time.elapsed().as_millis() as u32
    }

    fn clock_micros(&self) -> u64 {
        self.start_time.elapsed().as_micros() as u64
    }

    fn read_params(&mut self, params: &mut Params) -> bool {
        let Some(stored) = (unsafe { PARAM_STORE }) else {
            return false;
        };
        *params = stored;
        true
    }

    fn write_params(&mut self, params: &Params) -> bool {
        unsafe {
            PARAM_STORE = Some(*params);
        }
        true
    }

    fn reboot(&mut self) -> bool {
        self.pending_reset_to_bootloader = Some(false);
        true
    }

    fn reboot_to_bootloader(&mut self) -> bool {
        self.pending_reset_to_bootloader = Some(true);
        true
    }

    fn run_deferred_board_actions(&mut self) {
        if self.pending_reset_to_bootloader.take().is_some() {
            let mut delay = Delay;
            delay.delay_ms(20u32);
            stm_32::cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

impl Board {
    pub fn new() -> (Board, PixRacerProServoMonstrosity) {
        let p: EMBASSY_Peripherals = embassy_stm32::init(clock_config(24));

        let start_time = embassy_time::Instant::now();

        // SPI1 (ROSflight uses for internal ICM, unused here)
        let mut spi1_config: embassy_stm32::spi::Config = spi::Config::default();
        spi1_config.frequency = mhz(16); // Phil recommends not running over 4 Mbps
        spi1_config.mode = spi::MODE_3;
        spi1_config.bit_order = spi::BitOrder::MsbFirst;
        spi1_config.miso_pull = embassy_stm32::gpio::Pull::Up;
        let spi1 = spi::Spi::new(
            p.SPI1,
            p.PA5,
            p.PA7,
            p.PA6,
            p.DMA1_CH0,
            p.DMA1_CH1,
            BoardIrqs,
            spi1_config,
        );
        let spi1_bus = Mutex::new(spi1);
        let _spi1_bus = SPI1_BUS.init(spi1_bus);

        // SPI2 (internal DPS310)
        let mut spi2_config: embassy_stm32::spi::Config = spi::Config::default();
        spi2_config.frequency = mhz(16); // Phil recommends not running over 4 Mbps
        spi2_config.mode = spi::MODE_3;
        spi2_config.bit_order = spi::BitOrder::MsbFirst;
        spi2_config.miso_pull = embassy_stm32::gpio::Pull::Up;
        let spi2 = spi::Spi::new(
            p.SPI2,
            p.PB10, // Sck
            p.PB15, // Mosi
            p.PB14, // Miso
            p.DMA1_CH2,
            p.DMA1_CH3,
            BoardIrqs,
            spi2_config,
        );
        let spi2_bus = Mutex::new(spi2);
        let spi2_bus = SPI2_BUS.init(spi2_bus);

        // DPS310 Baro (Internal)
        let nss2 = Output::new(p.PD7, Level::High, Speed::Low);
        // these pins are generalized for the IC
        let drdy2 = ExtiInput::new(p.PD15, p.EXTI15, Pull::Down, BoardIrqs);
        let dps_dev = SpiDevice::new(spi2_bus, nss2);
        let dps_sensor = peripherals::dps310::Dps310Sensor {
            dev: dps_dev,
            drdy: drdy2,
            three_wire: false,
        };

        // I2C1 Bus (ist8308 Mag and MS4525 Pitot on external)
        let mut i2c_config = i2c::Config::default();
        i2c_config.scl_pullup = true;
        i2c_config.sda_pullup = true;
        i2c_config.frequency = Hertz(100_000);
        let i2c1 = i2c::I2c::new(
            p.I2C1, p.PB8, p.PB9, p.DMA2_CH2, p.DMA2_CH3, BoardIrqs, i2c_config,
        );
        let i2c1_bus = Mutex::new(i2c1);
        let i2c1_bus = I2C1_BUS.init(i2c1_bus);

        // IST8308 Magnetometer (External)
        let ist8303_sensor = peripherals::ist8308::Ist8308Sensor {
            dev: I2cDevice::new(i2c1_bus),
        };

        // MS4525 Pitot (External)
        let ms4525_sensor = peripherals::ms4525::Ms4525Sensor {
            dev: I2cDevice::new(i2c1_bus),
        };

        let llv3hp_sensor = peripherals::llv3hp::Llv3hpSensor {
            dev: I2cDevice::new(i2c1_bus),
        };

        // Companion Computer UART - Austin's documentation references uart3 instead of 2 for companion computer
        let mut uart3config = usart::Config::default();
        uart3config.rx_pull = Pull::Up;
        uart3config.baudrate = 921600;
        let uart3 = Uart::new(
            p.USART3,
            p.PD9,
            p.PD8,
            p.DMA2_CH4,
            p.DMA2_CH5,
            BoardIrqs,
            uart3config,
        )
        .unwrap();
        let (uart3_tx, uart3_rx) = uart3.split();

        let telem3_rx = peripherals::telem::TelemRx {
            uart_rx: uart3_rx,
            byte_processor: stm_32::peripherals::telem::BasicProcessor {},
        };

        let telem3_tx = peripherals::telem::TelemTx { uart_tx: uart3_tx };

        // VCP
        static EP_BUF_CELL: StaticCell<[u8; 256]> = StaticCell::new();
        let mut config = embassy_stm32::usb::Config::default();
        config.vbus_detection = true;
        let driver = Driver::new_fs(
            p.USB_OTG_FS,
            Irqs,
            p.PA12,
            p.PA11,
            EP_BUF_CELL.init([0u8; 256]),
            config,
        );
        let vcp = peripherals::vcp::Vcp {
            driver,
            byte_processor: stm_32::peripherals::vcp::BasicProcessor {},
        };

        // USART4 (external GPS)
        let mut uart4config = usart::Config::default();
        uart4config.baudrate = 9600u32;
        uart4config.rx_pull = Pull::Up;
        let uart4 = Uart::new(
            p.UART4,
            p.PA1,
            p.PA0,
            p.DMA2_CH6,
            p.DMA2_CH7,
            BoardIrqs,
            uart4config,
        )
        .unwrap();

        // UBlox NEO-M9N GNSS (External)
        let ublox_sensor = peripherals::ublox::UbloxSensor {
            uart: uart4,
            protocol: peripherals::ublox::Protocol::M8,
            baudrate: peripherals::ublox::Bitrate::Baud230400,
            nav_period_ms: 100u16,
        };
        let drdy_pps = ExtiInput::new(p.PG12, p.EXTI12, Pull::Down, BoardIrqs);
        let pps_sensor = peripherals::pps::PpsSensor { pps: drdy_pps };

        // S.Bus USART6
        // Sbus only uses Rx.
        let mut uart6config = usart::Config::default();
        uart6config.baudrate = 100000u32;
        uart6config.parity = usart::Parity::ParityEven;
        uart6config.stop_bits = usart::StopBits::STOP2;
        uart6config.invert_rx = true;
        uart6config.invert_tx = true;
        uart6config.data_bits = usart::DataBits::DataBits8;

        let usart6 = Uart::new(
            p.USART6,
            p.PC7,
            p.PC6,
            p.DMA1_CH4,
            p.DMA1_CH5,
            BoardIrqs,
            uart6config,
        )
        .unwrap();
        let (_uart6_tx, uart6_rx) = usart6.split();
        let sbus_rx = peripherals::sbus::SbusRC { uart: uart6_rx };

        // uSD SDMMC1
        let sdmmc1 = sdmmc::Sdmmc::new_4bit(
            p.SDMMC1,
            BoardIrqs,
            p.PC12,
            p.PD2,
            p.PC8,
            p.PC9,
            p.PC10,
            p.PC11,
            Default::default(),
        );

        // SPI5 (Internal BMI085)
        let mut spi5_config: embassy_stm32::spi::Config = spi::Config::default();
        spi5_config.frequency = mhz(2); // Phil recommends not running over 4 Mbps
        spi5_config.mode = spi::MODE_3;
        spi5_config.bit_order = spi::BitOrder::MsbFirst;
        spi5_config.miso_pull = embassy_stm32::gpio::Pull::Up;
        let spi5 = spi::Spi::new(
            p.SPI5,
            p.PF7,      // sck
            p.PF9,      // mosi
            p.PF8,      // miso
            p.DMA1_CH6, // tx_dma
            p.DMA1_CH7, // rx_dma
            BoardIrqs,
            spi5_config,
        );
        let spi5_ = Mutex::new(spi5);
        let spi5_bus = SPI5_BUS.init(spi5_);

        // BMI085 (Internal)
        let nss_bmi08x_a = Output::new(p.PF6, Level::High, Speed::Low); // Accel
        let drdy_bmi08x_a = ExtiInput::new(p.PF1, p.EXTI1, Pull::Down, BoardIrqs); // Accel
        let nss_bmi08x_g = Output::new(p.PF10, Level::High, Speed::Low); // Gyro
        let drdy_bmi08x_g = ExtiInput::new(p.PF3, p.EXTI3, Pull::Down, BoardIrqs); // Gyro
        let bmi08x_dev_a = SpiDevice::new(spi5_bus, nss_bmi08x_a);
        let bmi08x_dev_g = SpiDevice::new(spi5_bus, nss_bmi08x_g);
        let jumper: Output<'static> = Output::new(p.PF2, Level::High, Speed::Low); // Bridge pin
        let bmi08x_sensor = peripherals::bmi08x::Bmi08xSensor {
            dev_a: bmi08x_dev_a,
            dev_g: bmi08x_dev_g,
            drdy_a: drdy_bmi08x_a,
            drdy_g: drdy_bmi08x_g,
            jumper: jumper,
            range_a: peripherals::bmi08x::AccelRange::Bmi085(
                peripherals::bmi08x::AccelRange085::Max16g,
            ),
            range_g: peripherals::bmi08x::GyroRange::Max500dps,
            sample_rate: peripherals::bmi08x::SampleRate::Odr400Hz,
        };

        // Detect GPIO input.
        let usd_detect = embassy_stm32::gpio::Input::new(p.PG3, Pull::None); // PG3 is not connected
        let usd_card = peripherals::sd_card::SdCard {
            sdmmc: sdmmc1,
            detect: usd_detect,
        };

        // P1 Priority Task for Rx Telemetry
        interrupt::SAI1.set_priority(Priority::P0);
        let spawner1 = P1_EXECUTOR.start(interrupt::SAI1);
        spawn_task(&spawner1, peripherals::bmi08x::task(bmi08x_sensor));

        // P2 Priority Task for Gyros
        interrupt::SAI2.set_priority(Priority::P2);
        let spawner2 = P2_EXECUTOR.start(interrupt::SAI2);

        // P2 VCP Task (Telemetry alternate)
        spawn_task(&spawner2, peripherals::vcp::task(vcp));
        spawn_task(&spawner2, peripherals::telem::task_rx(telem3_rx));

        // P3 Priority Task for Polled Peripherals
        interrupt::SAI3.set_priority(Priority::P3);
        let spawner3 = P3_EXECUTOR.start(interrupt::SAI3);
        spawn_task(&spawner3, peripherals::ist8308::task(ist8303_sensor));
        spawn_task(&spawner3, peripherals::ms4525::task(ms4525_sensor));
        spawn_task(&spawner3, peripherals::dps310::task(dps_sensor));
        spawn_task(&spawner3, peripherals::ublox::task(ublox_sensor));
        spawn_task(&spawner3, peripherals::pps::task(pps_sensor));
        spawn_task(&spawner3, peripherals::sbus::task(sbus_rx));
        spawn_task(&spawner3, peripherals::llv3hp::task(llv3hp_sensor));

        // P4 Priority for Tx Telemetry
        interrupt::SAI4.set_priority(Priority::P4);
        let spawner4 = P4_EXECUTOR.start(interrupt::SAI4);
        spawn_task(&spawner4, peripherals::telem::task_tx(telem3_tx));
        spawn_task(&spawner4, peripherals::sd_card::task(usd_card));

        // SERVOS + TIMERS
        // There are only 7 available Servo Channels on the PixRacer Pro
        // TIM1
        let tim1_ch1_pin = PwmPin::<_, embassy_stm32::timer::Ch1>::new(p.PE9, OutputType::PushPull);
        let tim1_ch2_pin =
            PwmPin::<_, embassy_stm32::timer::Ch2>::new(p.PE11, OutputType::PushPull);
        let tim1_ch3_pin =
            PwmPin::<_, embassy_stm32::timer::Ch3>::new(p.PE13, OutputType::PushPull);
        let tim1_ch4_pin =
            PwmPin::<_, embassy_stm32::timer::Ch4>::new(p.PE14, OutputType::PushPull);

        // TIM4
        let tim4_ch2_pin =
            PwmPin::<_, embassy_stm32::timer::Ch2>::new(p.PD13, OutputType::PushPull);
        let tim4_ch3_pin =
            PwmPin::<_, embassy_stm32::timer::Ch3>::new(p.PD14, OutputType::PushPull);

        // TIM2
        let tim2_ch1_pin =
            PwmPin::<_, embassy_stm32::timer::Ch1>::new(p.PA15, OutputType::PushPull);

        let timer1 = SimplePwm::new(
            p.TIM1,
            Some(tim1_ch1_pin),
            Some(tim1_ch2_pin),
            Some(tim1_ch3_pin),
            Some(tim1_ch4_pin),
            Hertz::hz(400),
            Default::default(),
        );
        let timer4 = SimplePwm::new(
            p.TIM4,
            None,
            Some(tim4_ch2_pin),
            Some(tim4_ch3_pin),
            None,
            Hertz::hz(400),
            Default::default(),
        );
        let timer2 = SimplePwm::new(
            p.TIM2,
            Some(tim2_ch1_pin),
            None,
            None,
            None,
            Hertz::hz(400),
            Default::default(),
        );

        let timer1 = peripherals::pwm::TimerEnum::TIM1(timer1);
        let timer4 = peripherals::pwm::TimerEnum::TIM4(timer4);
        let timer2 = peripherals::pwm::TimerEnum::TIM2(timer2);

        let timers: [peripherals::pwm::TimerEnum; 3] = [timer1, timer2, timer4];

        let servos = peripherals::pwm::PixRacerProServoMonstrosity::with_timer_kinds_and_dma(
            timers,
            [
                (0, peripherals::pwm::TimerChannel::Ch1), // TIM1, channels 1-4
                (0, peripherals::pwm::TimerChannel::Ch2), // -
                (0, peripherals::pwm::TimerChannel::Ch3), // -
                (0, peripherals::pwm::TimerChannel::Ch4), // -
                (1, peripherals::pwm::TimerChannel::Ch1), // TIM2, channel 1
                (2, peripherals::pwm::TimerChannel::Ch2), // TIM4, channels 2 and 3
                (2, peripherals::pwm::TimerChannel::Ch3), // -
            ],
            [
                peripherals::pwm::PwmTimerBlockKind::StandardOnly,
                peripherals::pwm::PwmTimerBlockKind::StandardOnly,
                peripherals::pwm::PwmTimerBlockKind::StandardOnly,
            ],
            [None, None, None],
        );

        // Test PWM pins
        #[cfg_attr(feature = "scope-timing-pins", allow(unused_mut))]
        let mut test_pin_1 = Output::new(p.PD11, Level::Low, Speed::VeryHigh);
        #[cfg(not(feature = "scope-timing-pins"))]
        test_pin_1.set_high();
        #[cfg_attr(feature = "scope-timing-pins", allow(unused_mut))]
        let mut test_pin_2 = Output::new(p.PD12, Level::Low, Speed::VeryHigh);
        #[cfg(not(feature = "scope-timing-pins"))]
        test_pin_2.set_high();

        // Setup Probe GPIO's
        let probe = [
            Output::new(p.PG13, Level::Low, Speed::Low),
            Output::new(p.PG9, Level::Low, Speed::Low),
            Output::new(p.PG14, Level::Low, Speed::Low),
            // Output::new(p.PG0, Level::Low, Speed::Low), // unknown
        ];
        (
            Board {
                _probe: probe,
                start_time,
                test_pin_1,
                test_pin_2,
                pending_reset_to_bootloader: None,
                #[cfg(feature = "sensor-poll-diagnostics")]
                last_sbus_diag_ms: 0,
                #[cfg(feature = "sensor-poll-diagnostics")]
                sbus_rc_drains: 0,
                #[cfg(feature = "sensor-poll-diagnostics")]
                sbus_last_rc_status: 0,
                #[cfg(feature = "sensor-poll-diagnostics")]
                sbus_last_rc_lol: false,
            },
            servos,
        )
    }

    #[cfg(feature = "sensor-poll-diagnostics")]
    fn record_sbus_rc_drain(
        &mut self,
        rc: &Option<Result<veloxity_core::packets::RcPacket, errors::SensorError>>,
    ) {
        if let Some(result) = rc {
            self.sbus_rc_drains = self.sbus_rc_drains.wrapping_add(1);
            if let Ok(packet) = result {
                self.sbus_last_rc_status = packet.header.status as u32;
                self.sbus_last_rc_lol = packet.lol;
            }
        }
    }

    #[cfg(feature = "sensor-poll-diagnostics")]
    fn log_sbus_diagnostics_if_due(&mut self) {
        let now_ms = self.clock_millis();
        if now_ms.wrapping_sub(self.last_sbus_diag_ms) < 1_000 {
            return;
        }
        self.last_sbus_diag_ms = now_ms;

        let sbus = peripherals::sbus::diagnostics();
        veloxity_core::log_info!(
            "SBUS rx{} e{} sz{} n{} v{}",
            sbus.read_ok,
            sbus.read_err,
            sbus.last_read_size,
            sbus.size_25,
            sbus.valid_frame
        );
        veloxity_core::log_info!(
            "SBUS bh{} bf{} sig{} to{} dr{}",
            sbus.bad_header,
            sbus.bad_footer,
            sbus.signal,
            sbus.timeout,
            self.sbus_rc_drains
        );
        veloxity_core::log_info!(
            "SBUS st{} rst{} lol{}",
            sbus.last_status,
            self.sbus_last_rc_status,
            self.sbus_last_rc_lol as u8
        );
    }
}
