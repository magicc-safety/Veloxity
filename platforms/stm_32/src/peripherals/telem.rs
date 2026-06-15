use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_stm32::usart::UartTx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_time::Timer;

#[cfg(feature = "timing-diagnostics")]
use core::sync::atomic::{AtomicU32, Ordering};

use veloxity_core::comm::interface::EmbeddedComInterface;
use veloxity_core::errors;

pub static TX_BUFF_SIZE: usize = 4 * 2048;
pub static RX_BUFF_SIZE: usize = 4 * 2048;

pub static TELEM_TX: Pipe<CriticalSectionRawMutex, TX_BUFF_SIZE> = Pipe::new();
pub static TELEM_RX: Pipe<CriticalSectionRawMutex, RX_BUFF_SIZE> = Pipe::new();

#[cfg(feature = "timing-diagnostics")]
static TELEM_TX_DRAIN_READS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "timing-diagnostics")]
static TELEM_TX_DRAIN_BYTES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "timing-diagnostics")]
static TELEM_TX_UART_WRITES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "timing-diagnostics")]
static TELEM_TX_UART_BYTES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "timing-diagnostics")]
static TELEM_TX_UART_ERRORS: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "timing-diagnostics")]
pub fn take_tx_drain_diagnostics() -> (u32, u32, u32, u32, u32) {
    (
        TELEM_TX_DRAIN_READS.swap(0, Ordering::Relaxed),
        TELEM_TX_DRAIN_BYTES.swap(0, Ordering::Relaxed),
        TELEM_TX_UART_WRITES.swap(0, Ordering::Relaxed),
        TELEM_TX_UART_BYTES.swap(0, Ordering::Relaxed),
        TELEM_TX_UART_ERRORS.swap(0, Ordering::Relaxed),
    )
}

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
                #[cfg(feature = "timing-diagnostics")]
                {
                    TELEM_TX_DRAIN_READS.fetch_add(1, Ordering::Relaxed);
                    TELEM_TX_DRAIN_BYTES.fetch_add(n as u32, Ordering::Relaxed);
                }
                match self.uart_tx.write(&mut buf[0..n]).await {
                    Ok(()) => {
                        #[cfg(feature = "timing-diagnostics")]
                        {
                            TELEM_TX_UART_WRITES.fetch_add(1, Ordering::Relaxed);
                            TELEM_TX_UART_BYTES.fetch_add(n as u32, Ordering::Relaxed);
                        }
                    }
                    Err(_) => {
                        #[cfg(feature = "timing-diagnostics")]
                        TELEM_TX_UART_ERRORS.fetch_add(1, Ordering::Relaxed);
                        let _ = errors::TelemError::GenericTelemError("TelemTx Failed!");
                    }
                }
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
