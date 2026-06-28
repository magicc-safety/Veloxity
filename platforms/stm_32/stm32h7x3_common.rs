// ******************************************************************************
// * File     : platforms/stm_32/stm32h7x3_common.rs
// * Date     : June 28, 2026
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

use core::default::Default;
use core::option::Option::Some;
use embassy_stm32::{Config, rcc};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::InterruptExecutor;
use embassy_stm32::Peripherals as EMBASSY_Peripherals;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::exti;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::gpio::Pull;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::interrupt::Priority;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals as EMBASSY_peripherals;
use embassy_stm32::sdmmc;
use embassy_stm32::spi;
use embassy_stm32::spi::mode::Master as SpiMaster;
use embassy_stm32::time::Hertz;
use embassy_stm32::time::mhz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart;
use embassy_stm32::usart::Uart;
use embassy_stm32::usb;
use embassy_stm32::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
pub static SPI1_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async, SpiMaster>>> =
    StaticCell::new();
pub static SPI2_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async, SpiMaster>>> =
    StaticCell::new();
pub static SPI3_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async, SpiMaster>>> =
    StaticCell::new();
pub static SPI4_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async, SpiMaster>>> =
    StaticCell::new();
pub static SPI5_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async, SpiMaster>>> =
    StaticCell::new();
pub static SPI6_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async, SpiMaster>>> =
    StaticCell::new();

pub static I2C1_BUS: StaticCell<Mutex<CriticalSectionRawMutex, i2c::I2c<'static, Async, i2c::mode::Master>>> =
    StaticCell::new();
pub static I2C2_BUS: StaticCell<Mutex<CriticalSectionRawMutex, i2c::I2c<'static, Async, i2c::mode::Master>>> =
    StaticCell::new();
pub static I2C3_BUS: StaticCell<Mutex<CriticalSectionRawMutex, i2c::I2c<'static, Async, i2c::mode::Master>>> =
    StaticCell::new();
pub static I2C4_BUS: StaticCell<Mutex<CriticalSectionRawMutex, i2c::I2c<'static, Async, i2c::mode::Master>>> =
    StaticCell::new();

bind_interrupts!(struct BoardIrqs {
    I2C1_EV => i2c::EventInterruptHandler<EMBASSY_peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<EMBASSY_peripherals::I2C1>;
    I2C2_EV => i2c::EventInterruptHandler<EMBASSY_peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<EMBASSY_peripherals::I2C2>;
    I2C3_EV => i2c::EventInterruptHandler<EMBASSY_peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<EMBASSY_peripherals::I2C3>;
    I2C4_EV => i2c::EventInterruptHandler<EMBASSY_peripherals::I2C4>;
    I2C4_ER => i2c::ErrorInterruptHandler<EMBASSY_peripherals::I2C4>;
    USART1 => usart::InterruptHandler<EMBASSY_peripherals::USART1>;
    USART2 => usart::InterruptHandler<EMBASSY_peripherals::USART2>;
    USART3 => usart::InterruptHandler<EMBASSY_peripherals::USART3>;
    USART6 => usart::InterruptHandler<EMBASSY_peripherals::USART6>;
    UART4 => usart::InterruptHandler<EMBASSY_peripherals::UART4>;
    UART5 => usart::InterruptHandler<EMBASSY_peripherals::UART5>;
    UART7 => usart::InterruptHandler<EMBASSY_peripherals::UART7>;
    UART8 => usart::InterruptHandler<EMBASSY_peripherals::UART8>;
    SDMMC1 => sdmmc::InterruptHandler<EMBASSY_peripherals::SDMMC1>;
    DMA1_STREAM0 => dma::InterruptHandler<EMBASSY_peripherals::DMA1_CH0>;
    DMA1_STREAM1 => dma::InterruptHandler<EMBASSY_peripherals::DMA1_CH1>;
    DMA1_STREAM2 => dma::InterruptHandler<EMBASSY_peripherals::DMA1_CH2>;
    DMA1_STREAM3 => dma::InterruptHandler<EMBASSY_peripherals::DMA1_CH3>;
    DMA1_STREAM4 => dma::InterruptHandler<EMBASSY_peripherals::DMA1_CH4>;
    DMA1_STREAM5 => dma::InterruptHandler<EMBASSY_peripherals::DMA1_CH5>;
    DMA1_STREAM6 => dma::InterruptHandler<EMBASSY_peripherals::DMA1_CH6>;
    DMA1_STREAM7 => dma::InterruptHandler<EMBASSY_peripherals::DMA1_CH7>;
    DMA2_STREAM0 => dma::InterruptHandler<EMBASSY_peripherals::DMA2_CH0>;
    DMA2_STREAM1 => dma::InterruptHandler<EMBASSY_peripherals::DMA2_CH1>;
    DMA2_STREAM2 => dma::InterruptHandler<EMBASSY_peripherals::DMA2_CH2>;
    DMA2_STREAM3 => dma::InterruptHandler<EMBASSY_peripherals::DMA2_CH3>;
    DMA2_STREAM4 => dma::InterruptHandler<EMBASSY_peripherals::DMA2_CH4>;
    DMA2_STREAM5 => dma::InterruptHandler<EMBASSY_peripherals::DMA2_CH5>;
    DMA2_STREAM6 => dma::InterruptHandler<EMBASSY_peripherals::DMA2_CH6>;
    DMA2_STREAM7 => dma::InterruptHandler<EMBASSY_peripherals::DMA2_CH7>;
    EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
    EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
    EXTI2 => exti::InterruptHandler<interrupt::typelevel::EXTI2>;
    EXTI3 => exti::InterruptHandler<interrupt::typelevel::EXTI3>;
    EXTI4 => exti::InterruptHandler<interrupt::typelevel::EXTI4>;
    EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
});

// USB Interrupt
bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<EMBASSY_peripherals::USB_OTG_FS>;
});

// Executor Interrupts
// Use SAI1,2,3,4 as interrupt vectors since we are not using audio
// 1-4 are only conciedntally the same as I picked for the interrupt levels

static P0_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt]
unsafe fn SDMMC2() {
    unsafe { P0_EXECUTOR.on_interrupt() };
}

static P1_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt]
unsafe fn SAI1() {
    unsafe { P1_EXECUTOR.on_interrupt() };
}

static P2_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt]
unsafe fn SAI2() {
    unsafe { P2_EXECUTOR.on_interrupt() };
}

static P3_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt]
unsafe fn SAI3() {
    unsafe { P3_EXECUTOR.on_interrupt() };
}

static P4_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
#[interrupt]
unsafe fn SAI4() {
    unsafe { P4_EXECUTOR.on_interrupt() };
}

pub fn clock_config(mhz: u32) -> Config {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1); // (64 mHz, not used)

        // config.rcc.csi = true;
        // Select External 8MHz clock for Nucleo-H753ZI board
        // Select External 50MHz clock For Varmint
        config.rcc.hse = Some(rcc::Hse {
            freq: embassy_stm32::time::Hertz(mhz * 1_000_000), // HSE OSC
            mode: rcc::HseMode::Oscillator,
        });

        // This needs to be generalized for odd mhz
        let mut hsi_prediv = PllPreDiv::DIV4; // Default 8MHz case

        if mhz == 4 {
            hsi_prediv = PllPreDiv::DIV2;
        } else if mhz == 8  {
            hsi_prediv = PllPreDiv::DIV4;
        } else if mhz == 24  {
            hsi_prediv = PllPreDiv::DIV12;
        } else if mhz == 50  {
            hsi_prediv = PllPreDiv::DIV25;
        } else if mhz == 64  {
            hsi_prediv = PllPreDiv::DIV32;
        } else {
            //self::panic!("HSI OSC MHz value");
        }

        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,   // 50MHz
            prediv: hsi_prediv,       // 50MHz OSC / 25 = 2 MHz
            mul: PllMul::MUL400,      // 800 MHz
            divp: Some(PllDiv::DIV2), // 400 MHz for System Clock
            divq: Some(PllDiv::DIV8), // 100 MHz for SDMMC
            divr: Some(PllDiv::DIV2), // 400 MHz (not used)
        });

        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSE,    // 50MHz
            prediv: hsi_prediv,        // 50MHz OSC / 25 = 2 MHz
            mul: PllMul::MUL240,       // 480 MHz
            divp: Some(PllDiv::DIV30), // 16 MHz for SPI 1,2,3
            divq: Some(PllDiv::DIV30), // 16 MHz for SPI 4,5, and FDCAN
            divr: Some(PllDiv::DIV5),  // 96 MHz (not used)
        });

        config.rcc.pll3 = Some(Pll {
            source: PllSource::HSE,    // 50MHz
            prediv: hsi_prediv,        // 50MHz OSC / 25 = 2 MHz
            mul: PllMul::MUL480,       // 960 MHz
            divp: Some(PllDiv::DIV48), // 20 MHz (not used)
            divq: Some(PllDiv::DIV20), // 48 MHz for USB
            divr: Some(PllDiv::DIV15), // 64 MHz for ADC
        });
        // System clock MUX
        config.rcc.sys = Sysclk::PLL1_P; // Select PLL1_P for the System clock (400 MHz, see above)
        // D1CPRE Prescaler
        config.rcc.d1c_pre = AHBPrescaler::DIV1; // 400 MHz
        // HPRE Prescaler
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 MHz
        // D1PPRE, D2PPRE1, D2PPRE2, D3PPRE
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 MHz APB1 Peripheral Clocks for USART 2,3,4,5,7,8, I2C 1,2,3
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 MHz APB2 Peripheral Clocks for USART 1,6
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 MHz APB3 Peripheral Clocks
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 MHz APB4 Peripheral Clocks
        // SYSTICK Clock Prescaler
        config.rcc.timer_prescaler = TimerPrescaler::DefaultX2; // 400 MHz
        // 48MHz Clock used by USB? maybe (and RNG maybe)
        // config.rcc.hsi48 = Some(Default::default()); // Used for RNG
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        }); // For USB
        // Analog Voltage Detector level ??? (for startup?)
        config.rcc.voltage_scale = VoltageScale::Scale1; // ???// 2.8V. Scale1 (2.1V) is what is in all the examples. PWR_CR1 ALS bits.??
        // ADC clock
        config.rcc.mux.adcsel = mux::Adcsel::PLL3_R; // 64 MHz
        // USB clock
        //config.rcc.mux.usbsel = mux::Usbsel::PLL3_Q; // 48 MHz
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
        // SDMMC clock
        config.rcc.mux.sdmmcsel = mux::Sdmmcsel::PLL1_Q; // 100MHz
        // I2C 1-3,5
        config.rcc.mux.i2c1235sel = mux::I2c1235sel::PCLK1; // 100MHz
        // RNG
        config.rcc.mux.rngsel = mux::Rngsel::HSI48; // 48 MHz
        // SPI 1-3
        config.rcc.mux.spi123sel = mux::Saisel::PLL2_P; // 16 MHz
        // SPI4,5
        config.rcc.mux.spi45sel = mux::Spi45sel::PLL2_Q; // 16 MHz
        // USART 1,6
        config.rcc.mux.usart16910sel = mux::Usart16910sel::PCLK2; // 100 MHz
        // USART 2-5,7,8
        config.rcc.mux.usart234578sel = mux::Usart234578sel::PCLK1; // 100 MHz
    }
    return config;
}
