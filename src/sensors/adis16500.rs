use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_stm32::gpio::Output;

use embassy_time::Timer;
use embassy_time::Duration;
use embassy_time::Instant;

use embassy_stm32::exti::ExtiInput;

use defmt::info;

use crate::sensors::{RosflightPacketHeader, ImuPacket};

use core::module_path;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embedded_hal_async::spi::SpiDevice as _;

use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::peripherals::TIM1;

// Device dependent 
const SPI_READ: u8 = 0x00;
const SPI_WRITE: u8 = 0x80;

// Registers
// const OFFSET_REG: u8 = 0x45;
// const WHO_AM_I_REG: u8 = 0x4F;

// const CFG_REG_A: u8 = 0x60;
// const CFG_REG_B: u8 = 0x61;
// const CFG_REG_C: u8 = 0x62;
// const INT_CTRL_REG: u8 = 0x63;
// const INT_SOURCE_REG: u8 = 0x64;
// const INT_THS_L_REG: u8 = 0x65;
// const INT_THS_H_REG: u8 = 0x66;
// const STATUS_REG: u8 = 0x67;
//const OUT_FLUX: u8 =  0x68;
// const OUT_TEMP: u8 =  0x6E;

// Chip ID
// const WHO_AM_I: u8 = 0x40;

pub static IMU_SIGNAL  : Signal<CriticalSectionRawMutex, ImuPacket> = Signal::<CriticalSectionRawMutex, ImuPacket>::new();

pub struct Adis16500Sensor
{
    pub dev: SpiDevice<'static, CriticalSectionRawMutex, spi::Spi<'static, Async>, Output<'static>>,
    pub sample_period: Duration, 
    pub drdy: ExtiInput<'static>,
    pub reset: Output<'static>,
    pub timer: SimplePwm<'static,TIM1 >,
}

impl Adis16500Sensor
{
    async fn read_register(&mut self, reg_addr: u8) -> u16
    {
        let tx = [reg_addr|SPI_READ, 0x00];
        let _ = self.dev.write(& tx).await;
        Timer::after_micros(100).await; // Required 16us delay till you can read again
        let tx = [0u8;2];  
        let mut rx = [0u8;2];  
        let _ = self.dev.transfer(&mut rx, & tx).await;
        Timer::after_micros(100).await; // Required 16us delay till you can read again
        rx[1] as u16 | ((rx[0] as u16)<<8)
    }

    async fn write_register(&mut self, reg_addr: u8, value: u16)
    {
        let lo = (value&0x00FF) as u8;
        let tx = [reg_addr|SPI_WRITE, lo];
        let _ = self.dev.write(& tx).await; // Soft Reset
        Timer::after_micros(100).await; // (100) Required 16us delay till you can read again

        let hi = ((value>>8)&0x00FF) as u8; //
        let tx = [(reg_addr+1)|SPI_WRITE, hi];
        let _ = self.dev.write(& tx).await; // Soft Reset
        Timer::after_micros(100).await; // (100) Required 16us delay till you can read again
    }

    pub async fn run(&mut self)
    {    
        self.reset.set_low(); // Hold in reset
        // Start external sync clock
        let mut sync_clk = self.timer.ch1();
        sync_clk.enable();
        sync_clk.set_duty_cycle_fraction(1,2); // .set_duty_cycle(ch1.max_duty_cycle()/2);
        Timer::after_micros(1000).await; 
   
        // Hardware reset
     //   self.reset.set_low();
     //   Timer::after_micros(100).await; // Stay low at least 10 us
        self.reset.set_high();
        Timer::after_millis(300).await; // Data sheet specifies 255ms for power-on startup empirically 300 is required

        // Check the hardware ID
        const ADIS16500_PROD_ID_ADDR: u8 = 0x72;
        const ADIS16500_PROD_ID:u16 = 0x4074;
        let prod_id = self.read_register(ADIS16500_PROD_ID_ADDR).await;
        if prod_id == ADIS16500_PROD_ID { info!("PROD_ID = {:#04X} success",prod_id); }
        else { info!("PROD_ID = {:#04X} failure should be {:#04X}",prod_id, ADIS16500_PROD_ID);}
  
        const ADIS16500_FILT_CTRL:u8 = 0x5C; // shift so we can or the data into the first 16 bit packet
        // [15:3] not used
        // [2:0] 0 no digital filter default)
        self.write_register(ADIS16500_FILT_CTRL, 0).await;

        const ADIS16500_DEC_RATE: u8 = 0x64; // decimation
        // [15:11] don't care
        // [10:0] decimation rate minus 1, e.g., use 5-1 = 4
        let dec_rate :u16 = ( (self.sample_period.as_micros() as u16))/500u16 - 1u16; // decimation rate:  0 for 2000 Hz, 2000/400-1 = 4 for 400 Hz. 
        self.write_register(ADIS16500_DEC_RATE, dec_rate).await;

        info!("Dec rate = {:#04X}",dec_rate);

        // Miscellaneous Control Register (MSC_CTRL)
        const ADIS16500_MSC_CTRL:u8 =  0x60;
        // [15:10] 0's unused
        // [9] 1 32-bit burst data (default = 0)
        // [8] 0 burst data has gyro and accel data (default = 0)

        // [7] 1 enable linear acceleration compensation for gyros (default  0)
        // [6] 0 point of percussion alignment
        // [5] 0 always zero
        // [4] 0 wide sensor bandwidth (default)

        // [3:2] 01 Direct Input Sync Mode
        // [1] 0 falling edge sync (default =0)
        // [0] 1 active high when data is valid (default is 0, low)
        // 0b0000 0010 1000 0101 = 0x0285
     
        if dec_rate == 0 { // 2000Hz, sample rate, use 16-bit data mode
            self.write_register(ADIS16500_MSC_CTRL, 0x0085).await; // values 0b0000 0000 1000 0101 = 0x0085
        } else {                                      // use 32-bit data mode
            self.write_register(ADIS16500_MSC_CTRL, 0x0285).await; // values 0b0000 0010 1000 0101 = 0x0285
        }

        const ADIS16500_DIAG_STAT: u8 =  0x02;
        let diag_stat = self.read_register(ADIS16500_DIAG_STAT).await;

        if diag_stat == 0 { info!("Diag Stat = {:#04X} success",diag_stat); }
        else { info!("Diag Stat = {:#04X} failure should be {:#04X}",diag_stat, 0u16);}

        const BURST_READ:u8 = 0x68;
        loop {
            if dec_rate == 0 { // 2000Hz, sample rate, use 16-bit data mode
                const ADIS_BUFFBYTES16:usize = 22;
                self.drdy.wait_for_rising_edge().await;
                let timestamp = Instant::now();
                let mut rx = [0u8; ADIS_BUFFBYTES16];
                let mut tx = [0u8; ADIS_BUFFBYTES16];
                tx[0] = BURST_READ|SPI_READ;
                let _ = self.dev.transfer(&mut rx, &tx).await; 

                let rx_u16 = rx.map(|x| x as u16);
                let rx_u16_subarray = &rx_u16[2..ADIS_BUFFBYTES16-2];
                let checksum : u16 = rx_u16_subarray.iter().sum();

                let mut data = [0i16; ADIS_BUFFBYTES16/2];
                for (i, x) in data.iter_mut().enumerate() {
                    *x = ((rx[2 * i] as i16) << 8) | ((rx[2 * i + 1] as i16) & 0x00FF);
                }
 
                if checksum == data[10] as u16
                {
                    let status: u16 = data[1] as u16;  
                    if status == 0 {
                        let gyro = [
                            -f64::from(data[2]) * 0.001745329251994,  
                            -f64::from(data[3]) * 0.001745329251994,  
                             f64::from(data[4]) * 0.001745329251994 ];
                        let accel = [
                            -f64::from(data[5]) * 0.01225,  
                            -f64::from(data[6]) * 0.01225,  
                             f64::from(data[7]) * 0.01225 ];
                        let temperature = f32::from(data[8])*0.1; // + 273.15
                        let seq = data[9] as u16; // sequence counter    
                        let header = RosflightPacketHeader{timestamp, status};
                        let imu_packet =ImuPacket {header, accel, gyro, temperature, seq};
                        IMU_SIGNAL.signal(imu_packet);
                     }
                }
            } else {
                const ADIS_BUFFBYTES32:usize = 34;
                self.drdy.wait_for_rising_edge().await;
                let timestamp = Instant::now();
                let mut rx = [0u8; ADIS_BUFFBYTES32];
                let mut tx = [0u8; ADIS_BUFFBYTES32];
                tx[0] = BURST_READ|SPI_READ;
                let _ = self.dev.transfer(&mut rx, &tx).await; 

                let rx_u16 = rx.map(|x| x as u16);
                let rx_u16_subarray = &rx_u16[2..ADIS_BUFFBYTES32-2];
                let checksum : u16 = rx_u16_subarray.iter().sum();

                let mut data = [0u16; ADIS_BUFFBYTES32/2];
                for (i, x) in data.iter_mut().enumerate() {
                    *x = ((rx[2 * i] as u16) << 8) | ((rx[2 * i + 1] as u16) & 0x00FF);
                }
                if checksum == data[16] as u16
                {
                    let status: u16 = data[1] as u16;    
                    if status == 0 {
                        let gyros_sf: f64 =  0.001745329251994f64/f64::from(1u32 << 16);
                        let gyro = [
                            -f64::from( ((data[2] as u32) | ((data[3] as u32)<<16)) as i32 )*gyros_sf,  
                            -f64::from( ((data[4] as u32) | ((data[5] as u32)<<16)) as i32 )*gyros_sf,  
                             f64::from( ((data[6] as u32) | ((data[7] as u32)<<16)) as i32 )*gyros_sf
                        ];
                        let accel_sf: f64 =  0.012254f64/f64::from(1u32 << 16);
                            
                        let accel = [
                            -f64::from( ((data[8]  as u32) | ((data[9 ] as u32)<<16)) as i32 )*accel_sf,  
                            -f64::from( ((data[10] as u32) | ((data[11] as u32)<<16)) as i32 )*accel_sf,  
                             f64::from( ((data[12] as u32) | ((data[13] as u32)<<16)) as i32 )*accel_sf 
                        ];
                        let temperature = f32::from(data[14] as i16)*0.1; // + 273.15
                        let seq = data[15]; // sequence counter    
                        let header = RosflightPacketHeader{timestamp, status: seq};
                        let imu_packet =ImuPacket {header, accel, gyro, temperature, seq};
                        IMU_SIGNAL.signal(imu_packet);
                    }
                }
            }    
        }
    }
  }


#[embassy_executor::task]
pub async fn task(mut adis: Adis16500Sensor) 
{
    adis.run().await;
}

