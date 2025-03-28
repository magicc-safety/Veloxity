#![allow(unused)]

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

use embassy_sync::channel::Channel;
use embassy_sync::pipe::Pipe;
use embassy_stm32::usart::UartRx;

use embassy_stm32::usart::UartTx;

//use heapless::String;
use core::str;

static TX_BUFF_SIZE: usize = 1024;
static RX_BUFF_SIZE: usize = 512;

pub static TELEM_TX  : Pipe<CriticalSectionRawMutex, TX_BUFF_SIZE> = Pipe::new();
pub static TELEM_RX  : Pipe<CriticalSectionRawMutex, RX_BUFF_SIZE> = Pipe::new();

// pub static TELEM_TX  : Channel<CriticalSectionRawMutex, u8, 2048> = Channel::<CriticalSectionRawMutex, u8, 2048>::new();
// pub static TELEM_RX  : Channel<CriticalSectionRawMutex, u8, 2048> = Channel::<CriticalSectionRawMutex, u8, 2048>::new();

pub struct TelemTx {
    pub uart_tx: UartTx<'static, Async>,
}

pub struct TelemRx {
    pub uart_rx: UartRx<'static, Async>,
}

impl TelemTx
{
    pub async fn run(&mut self)
    {
         loop{
            let mut buf = [0u8;TX_BUFF_SIZE]; // read up to the whole buffer
            let n = TELEM_TX.read(&mut buf).await; 
            if n> 0 && n<=TX_BUFF_SIZE
            {
                //info!("{} {}",n, str::from_utf8(&buf[0..n]).unwrap());
                let result = self.uart_tx.write(&mut buf[0..n]).await;
                match result {
                    Err(error) => info!("{:?}",error),
                    Ok(_) => {}
                }
            }   
         }
        // Version using channel:
        // loop{
        //     let ch = TELEM_TX.receive().await;
        //     self.uart_tx.write(&[ch]).await;             
        // }
    }
}

impl TelemRx
{
    pub async fn run(&mut self)
    {
        let mut buf = [0u8;RX_BUFF_SIZE]; // Read as much as we can.
        loop {
            let result = self.uart_rx.read_until_idle(&mut buf).await;
            match result {
                Err(_) => { },
                Ok(n) => 
                { 
                    if n>0 && n<=RX_BUFF_SIZE 
                    {
                        TELEM_RX.write_all(&buf[0..n]).await;
                    } 
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task_rx(mut telem_rx: TelemRx) 
{
    telem_rx.run().await;
}

#[embassy_executor::task]
pub async fn task_tx(mut telem_tx: TelemTx) 
{
    info!("task_tx");
    telem_tx.run().await;
}