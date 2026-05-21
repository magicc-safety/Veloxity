use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_stm32::usart::UartTx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_time::Timer;

use voloxide_core::comm::interface::EmbeddedComInterface;
use voloxide_core::errors;

pub static TX_BUFF_SIZE: usize = 4 * 2048;
pub static RX_BUFF_SIZE: usize = 4 * 2048;

pub static TELEM_TX: Pipe<CriticalSectionRawMutex, TX_BUFF_SIZE> = Pipe::new();
pub static TELEM_RX: Pipe<CriticalSectionRawMutex, RX_BUFF_SIZE> = Pipe::new();

pub struct BasicProcessor;

impl EmbeddedComInterface for BasicProcessor {
    async fn process_bytes(&mut self, buf: &[u8], num_bytes: usize) {
        TELEM_RX.write_all(&buf[0..num_bytes]).await;
    }
}

pub struct TelemTx {
    pub uart_tx: UartTx<'static, Async>,
}

pub struct TelemRx<ECI: EmbeddedComInterface> {
    pub uart_rx: UartRx<'static, Async>,
    pub byte_processor: ECI,
}

impl TelemTx {
    pub async fn run(&mut self) {
        loop {
            let mut buf = [0u8; TX_BUFF_SIZE]; // read up to the whole buffer
            let n = TELEM_TX.read(&mut buf).await;
            if n > 0 && n <= TX_BUFF_SIZE {
                let _result = self
                    .uart_tx
                    .write(&mut buf[0..n])
                    .await
                    .map_err(|e| match e {
                        _ => errors::TelemError::GenericTelemError("TelemTx Failed!"),
                    });
            }
        }
    }
}

// is this impl how this is supposed to work?
impl<ECI: EmbeddedComInterface> TelemRx<ECI> {
    pub async fn run(&mut self) {
        let mut buf = [0u8; RX_BUFF_SIZE]; // Read as much as we can.
        loop {
            let result = self.uart_rx.read_until_idle(&mut buf).await;
            match result {
                Err(_) => {
                    Timer::after_millis(1).await;
                }
                Ok(n) => {
                    if n > 0 && n <= RX_BUFF_SIZE {
                        // added the above if statement to match phil's code
                        self.byte_processor.process_bytes(&buf, n).await;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task_rx(mut telem_rx: TelemRx<BasicProcessor>) {
    telem_rx.run().await;
}

#[embassy_executor::task]
pub async fn task_tx(mut telem_tx: TelemTx) {
    telem_tx.run().await;
}
