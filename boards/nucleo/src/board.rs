use voloxide_core::board::BoardIo;
use voloxide_core::errors;
use voloxide_core::params::Params;
use voloxide_core::pwm::{PwmDriver, PwmError};
use voloxide_core::sensors::SensorBus;

use embassy_time::Delay;
use stm_32::cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use stm_32::peripherals;
use stm_32::*;

include!("../../../platforms/stm_32/stm32h7x3_common.rs");

static mut PARAM_STORE: Option<Params> = None;

pub struct Board {
    _probe: [Output<'static>; 4],
    pub start_time: embassy_time::Instant,
    pending_reset_to_bootloader: Option<bool>,
}

pub struct BoardPwmDriver {
    servos: peripherals::pwm::ServoMonstrosity,
    enabled_chan_mask: u16,
}

impl BoardPwmDriver {
    pub fn new(servos: peripherals::pwm::ServoMonstrosity) -> Self {
        Self {
            servos,
            enabled_chan_mask: 0,
        }
    }
}

impl PwmDriver for BoardPwmDriver {
    fn len(&self) -> usize {
        self.servos.chan_list.len()
    }

    fn is_enabled(&self) -> bool {
        self.enabled_chan_mask == ((1 << self.len()) - 1)
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= self.len() {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.servos
            .enable(channel)
            .map_err(|_| PwmError::GenericError)?;
        self.enabled_chan_mask |= 1 << channel;
        Ok(())
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= self.len() {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.servos
            .disable(channel)
            .map_err(|_| PwmError::GenericError)?;
        self.enabled_chan_mask &= !(1 << channel);
        Ok(())
    }

    fn enable_all(&mut self) -> Result<(), PwmError> {
        for i in 0..self.len() {
            self.enable(i)?;
        }
        Ok(())
    }

    fn disable_all(&mut self) {
        for i in 0..self.len() {
            let _ = self.disable(i);
        }
    }

    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError> {
        if channel >= self.len() {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.servos
            .set_duty_cycle(channel, duty)
            .map_err(|_| PwmError::GenericError)
    }

    fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), PwmError> {
        self.servos
            .configure_output_rates(rates_hz)
            .map_err(timer_error_to_pwm_error)
    }

    fn flush<B: voloxide_core::board::BoardIo>(&mut self, _board: &mut B) {}

    fn send_commands<B: voloxide_core::board::BoardIo>(
        &mut self,
        board: &mut B,
        commands: &[f64],
    ) -> Result<(), PwmError> {
        self.servos
            .send_normalized_commands(commands)
            .map_err(timer_error_to_pwm_error)?;
        self.flush(board);
        Ok(())
    }
}

fn timer_error_to_pwm_error(error: peripherals::pwm::TimerError) -> PwmError {
    match error {
        peripherals::pwm::TimerError::ChanNotSupported => PwmError::ChannelOutOfRange,
        peripherals::pwm::TimerError::InvalidRate => PwmError::InvalidRate,
        peripherals::pwm::TimerError::UnsupportedProtocol => PwmError::UnsupportedProtocol,
        peripherals::pwm::TimerError::TimerNotSupported => PwmError::GenericError,
    }
}

impl BoardIo for Board {
    fn update_sensor_bus(&mut self, sensors: &mut SensorBus) {
        sensors.clear();
        sensors.imu = peripherals::bmi08x::IMU_SIGNAL.try_take();
        sensors.mag = peripherals::iis2mdc::MAG_SIGNAL.try_take();
        sensors.baro = peripherals::dps310::BARO_SIGNAL.try_take();
        sensors.pitot = peripherals::dlhrl20g::PITOT_SIGNAL.try_take();
        sensors.gnss = peripherals::ublox::GNSS_SIGNAL.try_take();
        sensors.rc = peripherals::sbus::RC_SIGNAL.try_take();
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        match peripherals::telem::TELEM_RX.try_read(buf) {
            Ok(n) => return Some(Ok(n)),
            Err(embassy_sync::pipe::TryReadError::Empty) => {
                return Some(Ok(0));
            }
        }
    }
    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
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
        Some(Ok(n))
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
    pub fn new() -> (Board, BoardPwmDriver) {
        let p: EMBASSY_Peripherals = embassy_stm32::init(clock_config(8));

        let start_time = embassy_time::Instant::now();

        // SPI1 Bus ///////////////////////////////////////////
        let mut spi1_config: embassy_stm32::spi::Config = spi::Config::default();
        spi1_config.frequency = mhz(1);
        spi1_config.mode = spi::MODE_3;
        spi1_config.bit_order = spi::BitOrder::MsbFirst;
        spi1_config.miso_pull = embassy_stm32::gpio::Pull::Up;
        let spi1 = spi::Spi::new(
            p.SPI1,
            p.PB3,
            p.PB5,
            p.PB4,
            p.DMA1_CH0,
            p.DMA1_CH1,
            spi1_config,
        );
        let spi1_bus = Mutex::new(spi1);
        let spi1_bus = SPI1_BUS.init(spi1_bus);

        // IIS2MDC Mag
        let nss1 = Output::new(p.PA4, Level::High, Speed::Low);
        let drdy1 = ExtiInput::new(p.PF3, p.EXTI3, Pull::Down);
        let iis_dev = SpiDevice::new(spi1_bus, nss1); // Todo implement new funciton
        let iis_sensor = peripherals::iis2mdc::Iis2mdcSensor {
            dev: iis_dev,
            drdy: drdy1,
        }; // Todo implement new funciton

        // DPS210 Baro
        let nss2 = Output::new(p.PC7, Level::High, Speed::Low);
        let drdy2 = ExtiInput::new(p.PG2, p.EXTI2, Pull::Down);
        let dps_dev = SpiDevice::new(spi1_bus, nss2);
        let dps_sensor = peripherals::dps310::Dps310Sensor {
            dev: dps_dev,
            drdy: drdy2,
            three_wire: true,
        }; // Todo implement new funciton

        // SPI2 Bus ///////////////////////////////////////////
        let mut spi2_config: embassy_stm32::spi::Config = spi::Config::default();
        spi2_config.frequency = mhz(1);
        spi2_config.mode = spi::MODE_3;
        spi2_config.bit_order = spi::BitOrder::MsbFirst;
        spi2_config.miso_pull = embassy_stm32::gpio::Pull::Up;
        let spi2 = spi::Spi::new(
            p.SPI2,
            p.PB10,
            p.PC3,
            p.PC2,
            p.DMA1_CH2,
            p.DMA1_CH3,
            spi2_config,
        );
        let spi2_bus = Mutex::new(spi2);
        let _spi2_bus = SPI2_BUS.init(spi2_bus);

        // I2C1 Bus  ///////////////////////////////////////////
        let mut i2c_config = i2c::Config::default();
        i2c_config.scl_pullup = true;
        i2c_config.sda_pullup = true;
        let i2c1 = i2c::I2c::new(
            p.I2C1,
            p.PB8,
            p.PB9,
            IrqsI2c1,
            p.DMA2_CH2,
            p.DMA2_CH3,
            Hertz(100_000),
            i2c_config,
        );
        let i2c1_bus = Mutex::new(i2c1);
        let i2c1_bus = I2C1_BUS.init(i2c1_bus);

        // DLHRL20G Pitot
        let drdy0 = ExtiInput::new(p.PA15, p.EXTI15, Pull::Down);
        let dlhr_dev = I2cDevice::new(i2c1_bus);
        let _dlhr_sensor = peripherals::dlhrl20g::DlhrL20GSensor {
            dev: dlhr_dev,
            drdy: drdy0,
        };

        // Telemetry UART
        let mut uart2config = usart::Config::default();
        uart2config.baudrate = 921600;
        let usart2 = Uart::new(
            p.USART2,
            p.PD6,
            p.PD5,
            Usart2Irqs,
            p.DMA2_CH4,
            p.DMA2_CH5,
            uart2config,
        )
        .unwrap();
        let (usart2_tx, usart2_rx) = usart2.split();

        let telem2_rx = peripherals::telem::TelemRx {
            uart_rx: usart2_rx,
            byte_processor: stm_32::peripherals::telem::BasicProcessor {},
        };

        let telem2_tx = peripherals::telem::TelemTx { uart_tx: usart2_tx };

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

        // P1 Priority Task for Rx Tememetry
        interrupt::SAI1.set_priority(Priority::P1);
        let spawner1 = P1_EXECUTOR.start(interrupt::SAI1);
        let _ = spawner1.spawn(peripherals::telem::task_rx(telem2_rx));
        spawner1.spawn(peripherals::vcp::task(vcp)).unwrap();

        //GPS USART7
        let mut uart7config = usart::Config::default();
        uart7config.baudrate = 9600u32;
        let uart7 = Uart::new(
            p.UART7,
            p.PE7,
            p.PE8,
            Uart7Irqs,
            p.DMA2_CH6,
            p.DMA2_CH7,
            uart7config,
        )
        .unwrap();
        let ublox_sensor = peripherals::ublox::UbloxSensor {
            uart: uart7,
            protocol: peripherals::ublox::Protocol::M8,
            baudrate: peripherals::ublox::Bitrate::Baud230400,
            nav_period_ms: 100u16,
        };
        let drdy_pps = ExtiInput::new(p.PE0, p.EXTI0, Pull::Down); // Gyro
        let pps_sensor = peripherals::pps::PpsSensor { pps: drdy_pps };

        // S.Bus USART1
        // Sbus only uses Rx.
        let mut uart1config = usart::Config::default();
        uart1config.baudrate = 100000u32;
        uart1config.parity = usart::Parity::ParityEven;
        uart1config.stop_bits = usart::StopBits::STOP2;
        uart1config.invert_rx = true;
        uart1config.invert_tx = true;
        uart1config.data_bits = usart::DataBits::DataBits8;

        let usart1 = Uart::new(
            p.USART1,
            p.PB7,
            p.PB6,
            Usart1Irqs,
            p.DMA1_CH4,
            p.DMA1_CH5,
            uart1config,
        )
        .unwrap();
        let (_uart1_tx, uart1_rx) = usart1.split();
        let sbus_rx = peripherals::sbus::SbusRC { uart: uart1_rx };

        // uSD SDMMC1
        let sdmmc1 = sdmmc::Sdmmc::new_4bit(
            p.SDMMC1,
            Sdmmc1Irqs,
            p.PC12,
            p.PD2,
            p.PC8,
            p.PC9,
            p.PC10,
            p.PC11,
            Default::default(),
        );

        // SPI4 Bus ///////////////////////////////////////////
        let mut spi4_config: embassy_stm32::spi::Config = spi::Config::default();
        spi4_config.frequency = mhz(2);
        spi4_config.mode = spi::MODE_3;
        spi4_config.bit_order = spi::BitOrder::MsbFirst;
        spi4_config.miso_pull = embassy_stm32::gpio::Pull::Up;
        let spi4 = spi::Spi::new(
            p.SPI4,
            p.PE2,
            p.PE6,
            p.PE5,
            p.DMA2_CH0,
            p.DMA2_CH1,
            spi4_config,
        );
        let spi4_ = Mutex::new(spi4);
        let spi4_bus = SPI4_BUS.init(spi4_);

        // BMI08x
        let nss_bmi08x_a = Output::new(p.PE3, Level::High, Speed::Low); // Accel
        let drdy_bmi08x_a = ExtiInput::new(p.PE4, p.EXTI4, Pull::Down); // Accel
        let nss_bmi08x_g = Output::new(p.PF8, Level::High, Speed::Low); // Gyro
        let drdy_bmi08x_g = ExtiInput::new(p.PF7, p.EXTI7, Pull::Down); // Gyro
        let bmi08x_dev_a = SpiDevice::new(spi4_bus, nss_bmi08x_a);
        let bmi08x_dev_g = SpiDevice::new(spi4_bus, nss_bmi08x_g);
        let jumper: Output<'static> = Output::new(p.PF15, Level::High, Speed::Low); // Bridge pin

        let bmi08x_sensor = peripherals::bmi08x::Bmi08xSensor {
            dev_a: bmi08x_dev_a,
            dev_g: bmi08x_dev_g,
            drdy_a: drdy_bmi08x_a,
            drdy_g: drdy_bmi08x_g,
            jumper: jumper,
            range_a: peripherals::bmi08x::AccelRange::Bmi088(
                peripherals::bmi08x::AccelRange088::Max24G,
            ),
            range_g: peripherals::bmi08x::GyroRange::Max500dps,
            sample_rate: peripherals::bmi08x::SampleRate::Odr400Hz,
        };

        // P2 Priority Task for Gyros
        interrupt::SAI2.set_priority(Priority::P2);
        let spawner2 = P2_EXECUTOR.start(interrupt::SAI2);
        spawner2
            .spawn(peripherals::bmi08x::task(bmi08x_sensor))
            .unwrap();

        // Detect GPIO input.
        let usd_detect = embassy_stm32::gpio::Input::new(p.PG3, Pull::None);
        let usd_card = peripherals::sd_card::SdCard {
            sdmmc: sdmmc1,
            detect: usd_detect,
        };

        // P3 Priority Task for Polled Peripherals
        interrupt::SAI3.set_priority(Priority::P3);
        let spawner3 = P3_EXECUTOR.start(interrupt::SAI3);
        //spawner3
        //    .spawn(peripherals::dlhrl20g::task(dlhr_sensor))
        //    .unwrap();
        spawner3
            .spawn(peripherals::iis2mdc::task(iis_sensor))
            .unwrap();
        spawner3
            .spawn(peripherals::dps310::task(dps_sensor))
            .unwrap();
        spawner3
            .spawn(peripherals::ublox::task(ublox_sensor))
            .unwrap();
        spawner3.spawn(peripherals::pps::task(pps_sensor)).unwrap();
        spawner3.spawn(peripherals::sbus::task(sbus_rx)).unwrap();

        // P4 Priority for Tx Telemetry
        interrupt::SAI4.set_priority(Priority::P4);
        let spawner4 = P4_EXECUTOR.start(interrupt::SAI4);
        spawner4
            .spawn(peripherals::telem::task_tx(telem2_tx))
            .unwrap();
        spawner4
            .spawn(peripherals::sd_card::task(usd_card))
            .unwrap();

        // SERVOS + TIMERS
        // TIM1
        let ch0_pin = PwmPin::new_ch1(p.PE9, OutputType::PushPull);
        let ch1_pin = PwmPin::new_ch2(p.PE11, OutputType::PushPull);
        let ch2_pin = PwmPin::new_ch3(p.PE13, OutputType::PushPull);
        let ch3_pin = PwmPin::new_ch4(p.PE14, OutputType::PushPull);
        // TIM4
        let ch4_pin = PwmPin::new_ch1(p.PD12, OutputType::PushPull);
        let ch5_pin = PwmPin::new_ch2(p.PD13, OutputType::PushPull);
        let ch6_pin = PwmPin::new_ch3(p.PD14, OutputType::PushPull);
        let ch7_pin = PwmPin::new_ch4(p.PD15, OutputType::PushPull);
        // TIM2
        let ch8_pin = PwmPin::new_ch1(p.PA0, OutputType::PushPull);
        let ch9_pin = PwmPin::new_ch4(p.PB11, OutputType::PushPull);
        // TIM3
        let ch10_pin = PwmPin::new_ch1(p.PC6, OutputType::PushPull);
        let ch11_pin = PwmPin::new_ch4(p.PB1, OutputType::PushPull);

        let timer1 = SimplePwm::new(
            p.TIM1,
            Some(ch0_pin),
            Some(ch1_pin),
            Some(ch2_pin),
            Some(ch3_pin),
            Hertz::hz(50),
            Default::default(),
        );
        let timer4 = SimplePwm::new(
            p.TIM4,
            Some(ch4_pin),
            Some(ch5_pin),
            Some(ch6_pin),
            Some(ch7_pin),
            Hertz::hz(50),
            Default::default(),
        );
        let timer2 = SimplePwm::new(
            p.TIM2,
            Some(ch8_pin),
            None,
            None,
            Some(ch9_pin),
            Hertz::hz(50),
            Default::default(),
        );
        let timer3 = SimplePwm::new(
            p.TIM3,
            Some(ch10_pin),
            None,
            None,
            Some(ch11_pin),
            Hertz::hz(50),
            Default::default(),
        );

        let timer1 = peripherals::pwm::TimerEnum::TIM1(timer1);
        let timer4 = peripherals::pwm::TimerEnum::TIM4(timer4);
        let timer2 = peripherals::pwm::TimerEnum::TIM2(timer2);
        let timer3 = peripherals::pwm::TimerEnum::TIM3(timer3);

        let timers: [peripherals::pwm::TimerEnum; 4] = [timer1, timer2, timer3, timer4];

        let mut servos = peripherals::pwm::ServoMonstrosity::new(
            timers,
            [
                (0, peripherals::pwm::TimerChannel::Ch1), //TIM1, channels 1-4
                (0, peripherals::pwm::TimerChannel::Ch2), // -
                (0, peripherals::pwm::TimerChannel::Ch3), // -
                (0, peripherals::pwm::TimerChannel::Ch4), // -
                (1, peripherals::pwm::TimerChannel::Ch1), //TIM2, channels 1, 4
                (1, peripherals::pwm::TimerChannel::Ch4), // -
                (2, peripherals::pwm::TimerChannel::Ch1), //TIM3, channels 1, 4
                (2, peripherals::pwm::TimerChannel::Ch4), // -
                (3, peripherals::pwm::TimerChannel::Ch1), //TIM4, channels 1-4
                (3, peripherals::pwm::TimerChannel::Ch2), // -
                (3, peripherals::pwm::TimerChannel::Ch3), // -
                (3, peripherals::pwm::TimerChannel::Ch4), // -
            ],
        );

        // disable all channels at start
        for i in 0..servos.len() {
            let _ = servos.disable(i);
        }

        // Setup Probe GPIO's
        let probe = [
            Output::new(p.PC0, Level::Low, Speed::Low),
            Output::new(p.PB2, Level::Low, Speed::Low),
            Output::new(p.PF2, Level::Low, Speed::Low),
            Output::new(p.PG0, Level::Low, Speed::Low),
        ];

        (
            Board {
                _probe: probe,
                start_time,
                pending_reset_to_bootloader: None,
            },
            BoardPwmDriver::new(servos),
        )
    }
}
