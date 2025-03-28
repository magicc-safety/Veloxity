//#![allow(unused)]
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_stm32::gpio::Output;

use embassy_time::Timer;
use embassy_time::Duration;

use embassy_stm32::exti::ExtiInput;

use embassy_time::with_timeout;
use crate::sensors::synch_at;

use defmt::info;

use crate::sensors::{RosflightPacketHeader, BaroPacket};

use core::module_path;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embedded_hal_async::spi::SpiDevice as _;

// Device dependent
const SPI_READ: u8 = 0x80;
const SPI_WRITE: u8 = 0x00;

pub static BARO_SIGNAL : Signal<CriticalSectionRawMutex, BaroPacket> = Signal::<CriticalSectionRawMutex, BaroPacket>::new();

pub struct Dps310Sensor
{
    pub dev: SpiDevice<'static, CriticalSectionRawMutex, spi::Spi<'static, Async>, Output<'static>>,
    pub drdy: ExtiInput<'static>,
    pub three_wire: bool,
}

fn compliment(x:u32, bits:u32) -> f64
{
    let mut x = x as i32;
    if ( x & (1i32 << (bits - 1)) ) != 0 { 
        x -= 1i32 << bits; 
    }
    f64::from(x)
}

const MEAS_CFG_REG: u8 = 0x08;
const ISR_REG:u8 = 0x0A;
const DPS310_READ_P_CMD: u8 = 0x00;
const DPS310_READ_T_CMD: u8 = 0x03;

const K1:f64 =  524288.0;     
//const K2:f64 =  1572864.0;    
//const K4:f64 =  3670016.0;  
const K8:f64 =  7864320.0; //    
//const K16:f64 =  253952.0; // with shift enabled in CFG_REG
//const K32:f64 =  516096.0; // with shift enabled in CFG_REG  
//const K64:f64 =  1040384.0; // with shift enabled in CFG_REG
//const K128:f64 =  2088960.0; // with shift enabled in CFG_REG 

impl Dps310Sensor
{
    async fn read_register(&mut self, reg_addr: u8) -> u8
    {
        let tx = [reg_addr|SPI_READ, 0x00];
        let mut rx = [0u8; 2];
        let _ = self.dev.transfer(&mut rx, & tx).await;
        rx[1]
    }

    // async fn read_register_2(&mut self, reg_addr: u8) 
    // -> Result< u8, ()>
    // {
    //     let tx = [reg_addr|SPI_READ, 0x00];
    //     let mut rx = [0u8; 2];
    //     let result = self.dev.transfer(&mut rx, & tx).await;
    //     match result {
    //         Ok(_) => Ok(rx[1]),
    //         Err(_) => Err(()),
    //     }
    // }

    async fn write_register(&mut self, reg_addr: u8, value: u8)
    {
        let tx = [reg_addr|SPI_WRITE, value];
        let _ = self.dev.write(& tx).await; // Soft Reset
    }

    pub async fn run( &mut self)
    {
        //////////////////////////////////////////////////////////////////////////////////////
        // Initialization
 
        // SOFT RESET
        const RESET_REG:u8 = 0x0C;
        self.write_register(  RESET_REG, 0x09).await;
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
        const CFG_REG:u8 = 0x09;   
        let three_wire_mode: u8 = if self.three_wire {0x01} else {0x00};
        let _ = self.write_register( CFG_REG,three_wire_mode|0xB0).await;
   
        // CHECK PRODUCT ID
        const PRODUCT_ID_REG: u8 = 0x0D;
        const PRODUCT_ID: u8 = 0x10;
        let id = self.read_register( PRODUCT_ID_REG).await;
        if id == PRODUCT_ID { info!("ID = {:#02x} success",id); }
        else { info!("ID = {:#02x} failure. Should be {:#02x}",id,PRODUCT_ID); }
 
        // match self.read_register_2( PRODUCT_ID_REG).await
        // {
        //     Ok(x) => {id = x},
        //     Err(())  => return,
        // }

        // CHECK IF CALIBRATION COEFFICIENTS ARE READY
        const COEF_READY: u8 = 0x80;
        let coef_rdy = self.read_register(  MEAS_CFG_REG).await;
        if (coef_rdy & COEF_READY) != 0x00 { info!("COEF_READY = {:#02x} success",coef_rdy); }
        else { info!("COEF_READY = {:#02x} failure. Should be {:#02x}",coef_rdy,0x80); }
 
        // READ CALIBRATION COEFFICIENTS
        const COEF_REG:u8 = 0x10;
        let tx  = [COEF_REG | SPI_READ, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let mut rx = [0u8;19];
        let _ = self.dev.transfer(&mut rx, & tx).await; 
 
        // move u8 date into u32 data for bit manipulation
        let buf = rx.map(|x| x as u32);
        // compute coefficint values in f64
        let mut cal =  [0f64;9];
        cal[0] = compliment((buf[1] << 4) | ((buf[2] >> 4) & 0x0F),12); // C0
        cal[1] = compliment(((buf[2] & 0x0F) << 8) | buf[3],12); // C1
        cal[2] = compliment((buf[4] << 12) | (buf[5] << 4) | ((buf[6] >> 4) & 0x0F),20); // C00
        cal[3] = compliment(((buf[6] & 0x0F) << 16) | (buf[7] << 8) | buf[8],20); // C10  
        cal[6] = compliment((buf[9] << 8)  | buf[10] ,16); // C01
        cal[7] = compliment((buf[11] << 8) | buf[12],16); // C11
        cal[4] = compliment((buf[13] << 8) | buf[14],16); // C20
        cal[8] = compliment((buf[15] << 8) | buf[16],16); // C21
        cal[5] = compliment((buf[17] << 8) | buf[18],16); // C30
        info!("Calibration Constants = {:?}",cal);

        // CHECK TEMPERATURE SOURCE
        const COEF_SRCE_REG: u8 =0x28;
        let temp_source = self.read_register( COEF_SRCE_REG).await & 0x80;
        info!("Temperature Source {:#02x}",temp_source); 

        // PRESSURE CONFIG
        const PRS_CFG_REG:u8 = 0x06;
        self.write_register(  PRS_CFG_REG, 0x03).await; // 8x oversampling
 
        // TEMPERATURE CONFIG
        const TMP_CFG_REG: u8 = 0x07;
        self.write_register(  TMP_CFG_REG, temp_source | 0x00 ).await; //no oversampling
 
        // Measurement Configuration
        // 7 - 	0, read only
        // 6 - 	0, read only
        // 5 - 	0, read only
        // 4 - 	0, read only
        // 3 - 	0, reserved
        // 2:0 - 	111, pressure and temperature continuous mode
        // Set to idle
        self.write_register( MEAS_CFG_REG,0x00).await;

    
        //////////////////////////////////////////////////////////////////////////////////////
        // Periodic Data Acquisition

        let mut traw_previous = 0_i32;
        let sample_period = Duration::from_hz(50);

        loop 
        {
            let timestamp = synch_at( sample_period );  
            Timer::at(timestamp).await; 
     
            // Start the Pressure read
            self.write_register(  MEAS_CFG_REG, 0x01).await;

            // Use DRDY signal for better robustness? otherwise, timeout at 14ms.
            if let Ok(()) = with_timeout(Duration::from_micros(14_000), self.drdy.wait_for_rising_edge()).await{}
            Timer::after_micros(20).await; // We need at least 14us delay here if running at 2 MHz, maybe because of the messy harness?

            // read status

            let mut status = (self.read_register( MEAS_CFG_REG ).await as u16)<<8;
            
            // read Pressure data
            let mut rx = [0u8; 4];
            let _ = self.dev.transfer(&mut rx, & [DPS310_READ_P_CMD|SPI_READ, 0,0,0]).await; 
            let praw = (((rx[1] as u32) << 24 | (rx[2] as u32) << 16 | (rx[3] as u32) << 8) as i32) >> 8;

            // Clear the ISR
            let _= self.read_register( ISR_REG ).await;

            // Start Temperature read    
            self.write_register(  MEAS_CFG_REG, 0x02).await;

            // Use DRDY signal if available, otherwise let it timeout
            if let Ok(()) = with_timeout(Duration::from_micros(20_000), self.drdy.wait_for_rising_edge()).await{}

            // read status
            status |= self.read_register( MEAS_CFG_REG ).await as u16;

            // read temperature data
            let mut rx = [0u8; 4];
            let _ = self.dev.transfer(&mut rx, & [DPS310_READ_T_CMD|SPI_READ, 0,0,0]).await; 
            // Clear the ISR
            let _ = self.read_register( ISR_REG ).await;

            // convert data to physical values 
            let traw: i32 = (((rx[1] as u32) << 24 | (rx[2] as u32) << 16 | (rx[3] as u32) << 8) as i32) >> 8;
            traw_previous += (traw-traw_previous)/16; // filter temperature a bit (1/127 is cutoff frequenc of 100Hz * (1/16)/(2*pi) around 1 sec to 1/e)
            let traw_f64 = f64::from(traw_previous)/K1;
            let temperature =  cal[0] * 0.5 + cal[1] * traw_f64; // K

            let praw_f64 = f64::from(praw)/K8;
            let pressure = 
                cal[2] + 
                praw_f64 * ( cal[3] + praw_f64 * (cal[4] + praw_f64 * cal[5])) + 
                traw_f64 * ( cal[6] + praw_f64 * (cal[7] + praw_f64 * cal[8])); // Pa
            
            // Pack-up and send out if good
            if status == 0xD0E0 
            {
                let header = RosflightPacketHeader{timestamp, status};
                let baro_packet = BaroPacket{header, pressure: pressure as f32, temperature: temperature as f32};
                BARO_SIGNAL.signal(baro_packet); // make data available for other tasks. 
            }
        }
 
    }

}

#[embassy_executor::task]
pub async fn task(mut dps: Dps310Sensor) 
{
     dps.run().await;
}

