use crate::params::Params;
use crate::board::Board;
use crate::board::nucleo_config::board_config;
use crate::sensors;

use cortex_m_rt::entry;
use defmt::*;

use embassy_stm32::dma::NoDma;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::interrupt::Priority;
use embassy_executor::InterruptExecutor;

use embassy_stm32::mode::Async;
use embassy_stm32::time::mhz;
use embassy_stm32::spi;
use embassy_stm32::i2c;
use embassy_stm32::usart;
use embassy_stm32::usart::BufferedUartTx;
use embassy_stm32::usart::Uart;
use embedded_io_async::BufRead;
use embassy_stm32::bind_interrupts;
use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::BufferedUart;
use embassy_time::Duration;
use embassy_stm32::gpio::{Output, Level, Speed};

use {defmt_rtt as _, panic_probe as _};


use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;

use static_cell::StaticCell;
use embassy_sync::mutex::Mutex;
 
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::Peripherals;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;

//use embassy_sync::pipe::{Pipe, TryReadError, TryWriteError};
use embassy_sync::channel::Channel;
use embassy_time::Instant;

use heapless::String;
use core::fmt::Write;


use embassy_stm32::gpio::OutputType;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};


use embedded_hal_async::spi::SpiDevice as _;

static SPI1_BUS: StaticCell<Mutex<CriticalSectionRawMutex,spi::Spi<'static, Async>>> = StaticCell::new();
static SPI2_BUS: StaticCell<Mutex<CriticalSectionRawMutex,spi::Spi<'static, Async>>> = StaticCell::new();
static I2C1_BUS: StaticCell<Mutex<CriticalSectionRawMutex,i2c::I2c<'static, Async>>> = StaticCell::new();

pub struct UartResources {
    pub uart: usart::BufferedUart<'static>,
}

bind_interrupts!(struct IrqsI2c1 {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

bind_interrupts!(struct Usart2Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

// Use SAI1,2,3,4 as interrupt vectors since we are not using audio
// 1-4 are only conciedntally the same as I picked for the interrupt levels

static P1_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt] unsafe fn SAI1() { P1_EXECUTOR.on_interrupt() }

static P2_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt] unsafe fn SAI2() { P2_EXECUTOR.on_interrupt() }

static P3_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt] unsafe fn SAI3() { P3_EXECUTOR.on_interrupt() }

static P4_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt] unsafe fn SAI4() { P4_EXECUTOR.on_interrupt() }

pub struct Nucleo {
    probe: [Output<'static>; 4],
    baro_data: sensors::BaroPacket,
    mag_data: sensors::MagPacket,
}


impl Board for Nucleo {
    /*
    TODO:
        * Check which functions actually need `&mut self` vs just passing &self
        * Check input types. Can we encode anything in Enums?
        * Check return types. Can we encode anything in Enums? For example, chage booleans to enums
     */

    // Setup
    fn init_board(&mut self) {
        // TODO
    }

    fn board_reset(&mut self, bootloader: bool) {
        // TODO
    }

    // Clock
    fn clock_millis(&self) -> u32 {
        0
    }

    fn clock_micros(&self) -> u64 {
        0
    }

    fn clock_delay(&self, milliseconds: u32) {
        // TODO 
    }

    // Serial
    fn serial_init(&self, baud_rate: u32, dev: u32) {
        // TODO
    }
     
    fn serial_write(&self, src: &[u8], qos: u8) {
        // TODO
    }

    fn serial_bytes_available(&self) -> u16 {
        0
    }

    fn serial_read(&self) -> u8 {
        0
    }
    
    fn serial_flush(&mut self) {
        // TODO
    }

    // Sensors
    fn sensors_init(&mut self) {
        // TODO
    }

    fn num_sensor_errors(&self) -> u16 {
        0
    }

    // IMU
    fn imu_has_new_data(&self) -> bool {
        false
    }

    fn imu_read(&self, accel: &mut [f32; 3], temperature: &mut f32, gyro: &mut [f32; 3], time: &mut u64) -> bool {
        false
    }
   
    fn imu_not_responding_error(&mut self) {
        // TODO
    }

    // Mag
    // fn mag_present(&self) -> bool {
    //     false
    // }

    // fn mag_has_new_data(&mut self) -> bool {
    //     let result = iis2mdc::MAG_SIGNAL.try_take();
    //     match result {
    //         Some(Ok(mag)) => {
    //             self.mag_data = mag;
    //             true
    //         },
    //         Some(Err(e)) => {
    //             trace!("Mag Error {}", e);
    //             false
    //         },
    //         None => false
    //     }
    // }

    // fn mag_read(&self, flux: &mut [f32; 3], temperature: &mut f32) -> bool {
    //     flux[0] = self.mag_data.flux[0]*1e6_f32;
    //     flux[1] = self.mag_data.flux[1]*1e-6_f32;
    //     flux[2] = self.mag_data.flux[2]*1e-6_f32;

    //     *temperature = self.mag_data.temperature;
    //     true
    // }

    fn mag_read(&self) -> Option<Result<sensors::MagPacket, sensors::SensorError>> {
        sensors::iis2mdc::MAG_SIGNAL.try_take()
    }

    // Baro
    // fn baro_present(&self) -> bool {
    //     false
    // }

    // fn baro_has_new_data(&mut self) -> bool {
    //     let result = dps310::BARO_SIGNAL.try_take();
    //     match result {
    //         Some(Ok(baro)) => {
    //             self.baro_data = baro;
    //             true
    //         },
    //         Some(Err(e)) => {
    //             trace!("Baro Error {}", e);
    //             false
    //         },
    //         None => false
    //     }
    // }
    
    // fn baro_read(&self, pressure: &mut f32, temperature: &mut f32) -> bool {
    //     *pressure = self.baro_data.pressure/1000_f32; 
    //     *temperature = self.baro_data.temperature;

    //     true
    // }

    fn baro_read(&self)-> Option<Result<sensors::BaroPacket, sensors::SensorError>> {
        sensors::dps310::BARO_SIGNAL.try_take()
    }

    // Pitot
    fn diff_pressure_present(&self) -> bool {
        false
    }

    fn diff_pressure_has_new_data(&self) -> bool {
        false
    }

    fn diff_pressure_read(&self, diff_pressure: &mut f32, temperature: &mut f32) -> bool {
        false
    }

    // Sonar
    fn sonar_present(&self) -> bool {
        false 
    }

    fn sonar_has_new_data(&self) -> bool {
        false
    }

    fn sonar_read(&self, range: &mut f32) -> bool {
        false
    }

    // GPS
    fn gnss_present(&self) -> bool {
        false
    }

    fn gnss_has_new_data(&self) -> bool {
        false
    }
    // fn gnss_read(&self, gnss: &mut GNSSData, gnss_full: &mut GNSSFull) -> bool;

    // Battery
    fn battery_present(&self) -> bool {
        false
    }

    fn battery_has_new_data(&self) -> bool {
        false
    }

    fn battery_read(&self, voltage: &mut f32, current: &mut f32) -> bool {
        false
    }
    
    fn battery_voltage_set_multiplier(&mut self, multiplier: f64) {
        // TODO
    }

    fn battery_current_set_multiplier(&mut self, multiplier: f64) {
        // TODO
    }

    // RC
    // fn rc_init(&mut self, rc_type: RcType);
    fn rc_lost(&self) -> bool {
        false
    }

    fn rc_has_new_data(&self) -> bool {
        false
    }
    
    fn rc_read(&self, chan: u8) -> f32 {
        0.0
    }

    // PWM
    fn pwm_init(&mut self, refresh_rate: u32, idle_pwm: u16) {
        // TODO
    }

    fn pwm_init_multi(&mut self, rate: &[f32], channels: u32) {
        // TODO
    }

    fn pwm_disable(&mut self) {
        // TODO
    }

    fn pwm_write(&mut self, channel: u8, value: f32) {
        // TODO
    }

    fn pwm_write_multi(&mut self, value: &[f32], channels: u32) {
        // TODO
    }

    // Non-volatile memory
    fn memory_init(&mut self) {
        // TODO
    }

    fn memory_read(&self, dest: &mut Params) -> bool {
        false
    }

    fn memory_write(&mut self, src: &Params) -> bool {
        false
    }

    // LEDs
    fn led0_on(&mut self) {
        // TODO
    }

    fn led0_off(&mut self) {
        // TODO
    }

    fn led0_toggle(&mut self) {
        // TODO
    }

    fn led1_on(&mut self) {
        // TODO
    }

    fn led1_off(&mut self) {
        // TODO
    }

    fn led1_toggle(&mut self) {
        // TODO
    }

    // Backup memory
    fn backup_memory_init(&mut self) {
        // TODO
    }

    fn backup_memory_read(&self, dest: &mut [u8]) -> bool {
        false
    }

    fn backup_memory_write(&mut self, src: &[u8]) {
        // TODO
    }

    fn backup_memory_clear(&mut self, len: usize) {
        // TODO
    }
}


impl Nucleo {
    fn probe_hi(&mut self, id: usize)
    {
        self.probe[id].set_high(); // so we can see something on the logic analyzer.
    }
    
    fn probe_lo(&mut self, id: usize)
    {
        self.probe[id].set_high(); // so we can see something on the logic analyzer.
    }
    
    fn probe_tog(&mut self, id: usize)
    {
        self.probe[id].toggle(); // so we can see something on the logic analyzer.
    }
    
    pub fn imu_read(&mut self)-> Option<sensors::ImuPacket>
    {
        sensors::adis16500::IMU_SIGNAL.try_take()
    }
    pub fn pitot_read(&mut self)-> Option<sensors::PitotPacket>
    {
        sensors::dlhrl20g::PITOT_SIGNAL.try_take()
    }

    // note: baro_read() is moved into the board implementation...
    // note: mag_read() is moved into the board implementation...
    
    pub fn telem_read(&mut self) -> Option<u8> // Read just one byte.
    {
        let mut buff = [0u8;1];
        let result = sensors::telem::TELEM_RX.try_read(&mut buff);
        match result {
            Err(_) => None,
            Ok(n) => Some(buff[0]) 
        }
    }
    
    pub fn telem_write(&mut self, mut buff: &[u8]) // write byte array
    {
        // This kind of stinks, find a better way.
        let len = buff.len();
        let mut n = 0;
        loop {
            let result = sensors::telem::TELEM_TX.try_write(&buff[n..len]);
            // For some stupid reason, pipes may not write everyting, even if there is room available.
            match result {
                Err(error) => info!("{:?}",error),
                Ok(wrote) => {
                    if(wrote==(len-n)) { break; }
                    else {n += wrote;}
                }
            }
        }
    } 

    pub fn new() -> Nucleo {
        let p: Peripherals = embassy_stm32::init(board_config());
        //let t = TestBoard{p: embassy_stm32::init(board_config())};
        // SPI1 Bus ///////////////////////////////////////////
        let mut spi1_config: embassy_stm32::spi::Config = spi::Config::default();
        spi1_config.frequency = mhz(1);
        spi1_config.mode = spi::MODE_3;
        spi1_config.bit_order = spi::BitOrder::MsbFirst;
        spi1_config.miso_pull = embassy_stm32::gpio::Pull::Up ;
        let spi1= spi::Spi::new(p.SPI1, p.PB3, p.PB5, p.PB4, p.DMA1_CH0, p.DMA2_CH0, spi1_config);
        let spi1_bus = Mutex::new(spi1);
        let spi1_bus = SPI1_BUS.init(spi1_bus);
        
        // IIS2MDC Mag
        let nss1 = Output::new(p.PA4, Level::High, Speed::Low);
        let drdy1 = ExtiInput::new(p.PF3, p.EXTI3, Pull::Down);
        let iis_dev = SpiDevice::new(spi1_bus, nss1); // Todo implement new funciton
        let iis_sensor = sensors::iis2mdc::Iis2mdcSensor{ dev: iis_dev, drdy: drdy1}; // Todo implement new funciton
    
        // DPS210 Baro
        let nss2 = Output::new(p.PC7, Level::High, Speed::Low);
        let drdy2 = ExtiInput::new(p.PG2, p.EXTI2, Pull::Down);
        let dps_dev = SpiDevice::new(spi1_bus, nss2);
        let dps_sensor = sensors::dps310::Dps310Sensor{ dev:dps_dev, drdy: drdy2 , three_wire: true}; // Todo implement new funciton

        // SPI2 Bus ///////////////////////////////////////////
        let mut spi2_config: embassy_stm32::spi::Config = spi::Config::default();
        spi2_config.frequency = mhz(1);
        spi2_config.mode = spi::MODE_3;
        spi2_config.bit_order = spi::BitOrder::MsbFirst;
        spi2_config.miso_pull = embassy_stm32::gpio::Pull::Up ;
        let spi2= spi::Spi::new(p.SPI2, p.PB10, p.PC3, p.PC2, p.DMA1_CH1, p.DMA2_CH1, spi2_config);
        let spi2_bus = Mutex::new(spi2);
        let spi2_bus = SPI2_BUS.init(spi2_bus);

        // ADIS16500 
        
        // TIMER
        let ch1_pin = PwmPin::new_ch1(p.PE9, OutputType::PushPull);
        let timer1 = SimplePwm::new(p.TIM1, Some(ch1_pin), None, None, None, Hertz::khz(2), Default::default());
        // The whole timer needs to be dedicated? There's probably a better way.

        let nss3 = Output::new(p.PG14, Level::High, Speed::Low);
        let drdy3 = ExtiInput::new(p.PG1, p.EXTI1, Pull::Down);
        let reset = Output::new(p.PE14, Level::High, Speed::Low);
        let adis_dev = SpiDevice::new(spi2_bus, nss3);
        let adis_sensor = sensors::adis16500::Adis16500Sensor{ dev: adis_dev, drdy: drdy3, reset, sample_period: Duration::from_hz(400), timer: timer1 }; // Todo implement new function
    
        // I2C1 Bus  ///////////////////////////////////////////
        let mut i2c_config = i2c::Config::default();
        i2c_config.scl_pullup = true;
        i2c_config.sda_pullup = true;
        let i2c1 = i2c::I2c::new(p.I2C1, p.PB8, p.PB9, IrqsI2c1, p.DMA1_CH2, p.DMA2_CH2, Hertz(100_000), i2c_config);
        let i2c1_bus = Mutex::new(i2c1);
        let i2c1_bus = I2C1_BUS.init(i2c1_bus);

        // DLHRL20G Pitot
        let drdy0 = ExtiInput::new(p.PA15, p.EXTI15, Pull::Down);
        let dlhr_dev = I2cDevice::new(i2c1_bus);
        let dlhr_sensor = sensors::dlhrl20g::DlhrL20GSensor{ dev: dlhr_dev, drdy: drdy0 };

        // Telemetry UART
        let mut uart2config = usart::Config::default();
        uart2config.baudrate = 921600;
        let mut usart2 = Uart::new(p.USART2, p.PD6, p.PD5, Usart2Irqs, p.DMA1_CH3, p.DMA2_CH3, uart2config).unwrap();
        let ( mut usart2_tx, mut usart2_rx) = usart2.split();

        let telem2_rx = sensors::telem::TelemRx{uart_rx: usart2_rx};
        let telem2_tx = sensors::telem::TelemTx{uart_tx: usart2_tx};

        // P1 Priority Task for Rx Tememetry
        interrupt::SAI1.set_priority(Priority::P1);
        let spawner1 =  P1_EXECUTOR.start(interrupt::SAI1);
        //spawner1.spawn(telem::task_rx(telem2_rx));

        // P2 Priority Task for Gyros
        interrupt::SAI2.set_priority(Priority::P2);
        let spawner2 =  P2_EXECUTOR.start(interrupt::SAI2);
        spawner2.spawn(sensors::adis16500::task(adis_sensor)).unwrap();

        // P3 Priority Task for Polled Sensors
        interrupt::SAI3.set_priority(Priority::P3);
        let spawner3 =  P3_EXECUTOR.start(interrupt::SAI3);
        spawner3.spawn(sensors::dlhrl20g::task(dlhr_sensor)).unwrap();
        spawner3.spawn(sensors::iis2mdc::task(iis_sensor)).unwrap();
        spawner3.spawn(sensors::dps310::task(dps_sensor)).unwrap();

        // P4 Priority for Tx Telemetry
        interrupt::SAI4.set_priority(Priority::P4);
        let spawner4 = P4_EXECUTOR.start(interrupt::SAI4);
        spawner4.spawn(sensors::telem::task_tx(telem2_tx));
    

        // Setup Probe GPIO's
        let probe = [ Output::new(p.PC6, Level::Low, Speed::Low),
        Output::new(p.PB15, Level::Low, Speed::Low),
        Output::new(p.PB12, Level::Low, Speed::Low),
        Output::new(p.PG3, Level::Low, Speed::Low)    ];
        Nucleo {probe
            , baro_data: sensors::BaroPacket {
                header: sensors::RosflightPacketHeader {
                    timestamp: Instant::from_micros(0)
                    , status: 0
                }
                , pressure: 0.0
                , temperature: 0.0
            }
            , mag_data: sensors::MagPacket {
                header: sensors::RosflightPacketHeader {
                    timestamp: Instant::from_micros(0)
                    , status: 0
                }
                , flux: [0.0, 0.0, 0.0]
                , temperature: 0.0
            }
        }
    }
}