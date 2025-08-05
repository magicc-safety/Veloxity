// /**
// ******************************************************************************
// * File     : nucleo.rs
// * Date     : May 8, 2025
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************
// **/
use rustflight_core::board::BoardTrait;
use rustflight_core::errors;
use rustflight_core::hlist_type;
use rustflight_core::packets;
use rustflight_core::sensorprocessors;

use stm_32::peripherals;
use stm_32::*;

include!("../../stm_32/stm32h7x3_common.rs");

pub struct Board {
    probe: [Output<'static>; 4],
    servos: peripherals::pwm::ServoMonstrosity,
}

impl BoardTrait for Board {
    type RawSensorSet = hlist_type![
        Option<Result<packets::ImuPacket, errors::SensorError>>,
        Option<Result<packets::MagPacket, errors::SensorError>>,
        Option<Result<packets::BaroPacket, errors::SensorError>>,
        Option<Result<packets::PitotPacket, errors::SensorError>>,
        Option<Result<packets::GNSSPacket, errors::SensorError>>,
        Option<Result<packets::RcPacket, errors::SensorError>>
    ];

    type ProcessedSensorSet = hlist_type![
        Option<packets::ImuPacket>,
        Option<packets::MagPacket>,
        Option<packets::BaroPacket>,
        Option<packets::PitotPacket>,
        Option<packets::GNSSPacket>,
        Option<packets::RcPacket>
    ];

    type ProcessorHList = hlist_type![
        sensorprocessors::PassthroughImuProcessor,
        sensorprocessors::PassthroughMagProcessor,
        sensorprocessors::PassthroughBaroProcessor,
        sensorprocessors::PassthroughPitotProcessor,
        sensorprocessors::PassthroughGNSSProcessor,
        sensorprocessors::PassthroughRcProcessor
    ];

    fn update_sensors(&mut self, sensors: &mut Self::RawSensorSet) {
        sensors.0 = peripherals::bmi08x::IMU_SIGNAL.try_take();
        sensors.1.0 = peripherals::iis2mdc::MAG_SIGNAL.try_take();
        sensors.1.1.0 = peripherals::dps310::BARO_SIGNAL.try_take();
        sensors.1.1.1.0 = peripherals::dlhrl20g::PITOT_SIGNAL.try_take();
        sensors.1.1.1.1.0 = peripherals::ublox::GNSS_SIGNAL.try_take();
        sensors.1.1.1.1.1.0 = peripherals::sbus::RC_SIGNAL.try_take();
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        match peripherals::telem::TELEM_RX.try_read(buf) {
            Ok(n) => return Some(Ok(n)),
            Err(_) => {
                return Some(Err(errors::TelemError::GenericTelemError(
                    "Error Reading Telem Packet",
                )));
            }
        }
    }
    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        let mut n = 0;
        //let len = byte_count;
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
}

/*
impl BoardTrait for Board {
    fn imu_read(&mut self) -> Option<Result<packets::ImuPacket, errors::SensorError>> {
        peripherals::bmi08x::IMU_SIGNAL.try_take()
    }

    fn mag_read(&mut self) -> Option<Result<packets::MagPacket, errors::SensorError>> {
        peripherals::iis2mdc::MAG_SIGNAL.try_take()
    }

    fn baro_read(&mut self) -> Option<Result<packets::BaroPacket, errors::SensorError>> {
        peripherals::dps310::BARO_SIGNAL.try_take()
    }

    fn diff_pressure_read(&mut self) -> Option<Result<packets::PitotPacket, errors::SensorError>> {
        peripherals::dlhrl20g::PITOT_SIGNAL.try_take()
    }

    fn sonar_read(&mut self) -> Option<Result<packets::RangePacket, errors::SensorError>> {
        None
    }

    fn gnss_read(&mut self) -> Option<Result<packets::GNSSPacket, errors::SensorError>> {
        peripherals::ublox::GNSS_SIGNAL.try_take()
    }

    fn battery_read(&mut self) -> Option<Result<packets::BatteryPacket, errors::SensorError>> {
        None
    }

    fn rc_read(&mut self) -> Option<Result<packets::RcPacket, errors::SensorError>> {
        peripherals::sbus::RC_SIGNAL.try_take()
    }

    fn attitude_read(&mut self) -> Option<Result<packets::AttitudePacket, errors::SensorError>> {
        None
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        match peripherals::telem::TELEM_RX.try_read(buf) {
            Ok(n) => return Some(Ok(n)),
            Err(_) => {
                return Some(Err(errors::TelemError::GenericTelemError(
                    "Error Reading Telem Packet",
                )));
            }
        }
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        let mut n = 0;
        //let len = byte_count;
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
}
*/

impl Board {
    fn probe_hi(&mut self, id: usize) {
        self.probe[id].set_high(); // so we can see something on the logic analyzer.
    }

    fn probe_lo(&mut self, id: usize) {
        self.probe[id].set_high(); // so we can see something on the logic analyzer.
    }

    fn probe_tog(&mut self, id: usize) {
        self.probe[id].toggle(); // so we can see something on the logic analyzer.
    }

    pub fn new() -> Board {
        let p: EMBASSY_Peripherals = embassy_stm32::init(clock_config(8));
        //let t = TestBoard{p: embassy_stm32::init(clock_config())};
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
        let spi2_bus = SPI2_BUS.init(spi2_bus);

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
        let dlhr_sensor = peripherals::dlhrl20g::DlhrL20GSensor {
            dev: dlhr_dev,
            drdy: drdy0,
        };

        // Telemetry UART
        let mut uart2config = usart::Config::default();
        uart2config.baudrate = 921600;
        let mut usart2 = Uart::new(
            p.USART2,
            p.PD6,
            p.PD5,
            Usart2Irqs,
            p.DMA2_CH4,
            p.DMA2_CH5,
            uart2config,
        )
        .unwrap();
        let (mut usart2_tx, mut usart2_rx) = usart2.split();

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
            config
        );
        let vcp = peripherals::vcp::Vcp {
            driver,
            byte_processor: stm_32::peripherals::vcp::BasicProcessor {},
        };

        // P1 Priority Task for Rx Tememetry
        interrupt::SAI1.set_priority(Priority::P1);
        let spawner1 = P1_EXECUTOR.start(interrupt::SAI1);
        spawner1.spawn(peripherals::telem::task_rx(telem2_rx));
        // TODO: What priority should VCP be?
        spawner1
            .spawn(peripherals::vcp::task(vcp))
            .unwrap();

        //GPS USART7
        let mut uart7config = usart::Config::default();
        uart7config.baudrate = 9600u32;
        let mut uart7 = Uart::new(
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

        let mut usart1 = Uart::new(
            p.USART1,
            p.PB7,
            p.PB6,
            Usart1Irqs,
            p.DMA1_CH4,
            p.DMA1_CH5,
            uart1config,
        )
        .unwrap();
        let (mut uart1_tx, mut uart1_rx) = usart1.split();
        let sbus_rx = peripherals::sbus::SbusRC { uart: uart1_rx };

        // uSD SDMMC1
        let mut sdmmc1 = sdmmc::Sdmmc::new_4bit(
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

        let bmi08x_sensor = peripherals::bmi08x::Bmi08xSensor {
            dev_a: bmi08x_dev_a,
            dev_g: bmi08x_dev_g,
            drdy_a: drdy_bmi08x_a,
            drdy_g: drdy_bmi08x_g,
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

        let mut timer1 = SimplePwm::new(
            p.TIM1,
            Some(ch0_pin),
            Some(ch1_pin),
            Some(ch2_pin),
            Some(ch3_pin),
            Hertz::hz(50),
            Default::default(),
        );
        let mut timer4 = SimplePwm::new(
            p.TIM4,
            Some(ch4_pin),
            Some(ch5_pin),
            Some(ch6_pin),
            Some(ch7_pin),
            Hertz::hz(50),
            Default::default(),
        );
        let mut timer2 = SimplePwm::new(
            p.TIM2,
            Some(ch8_pin),
            None,
            None,
            Some(ch9_pin),
            Hertz::hz(50),
            Default::default(),
        );
        let mut timer3 = SimplePwm::new(
            p.TIM3,
            Some(ch10_pin),
            None,
            None,
            Some(ch11_pin),
            Hertz::hz(50),
            Default::default(),
        );

        let mut timer1 = peripherals::pwm::TimerEnum::TIM1(timer1);
        let mut timer4 = peripherals::pwm::TimerEnum::TIM4(timer4);
        let mut timer2 = peripherals::pwm::TimerEnum::TIM2(timer2);
        let mut timer3 = peripherals::pwm::TimerEnum::TIM3(timer3);

        let mut timers: [peripherals::pwm::TimerEnum; 4] = [timer1, timer2, timer3, timer4];

        let mut servos: peripherals::pwm::ServoMonstrosity = peripherals::pwm::ServoMonstrosity {
            timers,
            chan_list: [
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
        };

        // disable all channels at start
        for i in 0..servos.len() {
            servos.disable(i);
        }

        // Setup Probe GPIO's
        let probe = [
            Output::new(p.PC0, Level::Low, Speed::Low),
            Output::new(p.PB2, Level::Low, Speed::Low),
            Output::new(p.PF2, Level::Low, Speed::Low),
            Output::new(p.PG0, Level::Low, Speed::Low),
        ];
        Board { probe, servos }
    }
}
