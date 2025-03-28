//#![allow(unused)]
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_stm32::i2c::I2c;

use embassy_stm32::mode::Async;

use embassy_time::Timer;
use embassy_time::Duration;

use embassy_stm32::exti::ExtiInput;

use embassy_time::with_timeout;
use crate::sensors::synch_at;

use crate::sensors::{RosflightPacketHeader, PitotPacket};


use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embedded_hal_async::i2c::I2c as _;

pub static PITOT_SIGNAL : Signal<CriticalSectionRawMutex, PitotPacket> = Signal::<CriticalSectionRawMutex, PitotPacket>::new();

pub struct DlhrL20GSensor
{
    pub dev: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    pub drdy: ExtiInput<'static>,
}

impl DlhrL20GSensor
{
    //pub async fn run( &mut self, mut i2c: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>)
    pub async fn run( &mut self )
    {
        const ADDRESS: u8 = 0x29;
        const START: u8 = 0xAC;

        let sample_period = Duration::from_hz(100);

        loop 
        {
            let timestamp = synch_at( sample_period );  
            Timer::at(timestamp).await;             // Wait for top of 100 Hz timer
     
            if let Ok(()) = self.dev.write(ADDRESS, &[START] ).await
            {
                if let Ok(()) = with_timeout(Duration::from_millis(100), self.drdy.wait_for_rising_edge()).await
                {
                    let mut data = [0u8;7];
                    if let Ok(()) = self.dev.read(ADDRESS,&mut data).await
                    {
                        let status = data[0] as u16;
                        let u32_pressure = u32::from(data[1])<<16 | u32::from(data[2])<<8 | u32::from(data[3]); 
                        let u32_temperature = u32::from(data[4])<<16 | u32::from(data[5])<<8 | u32::from(data[6]); 
                        
                        let fs = 5000.0;   // Pa, Full Scale pressure
                     
                        let pressure = 1.25 * fs * (f64::from(u32_pressure) / 16777216.0 - 0.1);         // Pa
                        let temperature = 125.0 * f64::from(u32_temperature) / 16777216.0 - 40.0 ; // C

                        //if status == 0x0040 
                        {
                            let header = RosflightPacketHeader{timestamp, status};
                            let pitot_packet = PitotPacket{header, pressure: pressure as f32, temperature: temperature as f32};
                            PITOT_SIGNAL.signal(pitot_packet); // make data available for other tasks. 
                        }                
                    }   
                }
            }     
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut dlhr: DlhrL20GSensor ) 
{
    dlhr.run().await;
}

