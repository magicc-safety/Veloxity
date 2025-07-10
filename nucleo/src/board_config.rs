// /**
// ******************************************************************************
// * File     : nucleo_config.rs
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
use core::default::Default;
use core::option::Option::Some;
use embassy_stm32::{Config, rcc};
use stm_32::*;

//
// Common settings for STM32H753 and STM32H743 boards
//

pub fn board_config() -> Config {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1); // (64 mHz, not used)

        config.rcc.csi = true;
        // Select External 8MHz clock for Nucleo-H753ZI board
        // Select External 50MHz clock For Varmint
        config.rcc.hse = Some(rcc::Hse {
            freq: embassy_stm32::time::Hertz(8_000_000), // 8MHz HSE OSC
            mode: rcc::HseMode::Oscillator,
        });

        config.rcc.pll1 = Some(Pll {
            // note: PllSource::HSI <-- internal oscillator (8 MHz)
            //       PllSource::HSE <-- external oscillator (8 MHz)
            source: PllSource::HSI,   // 8MHz
            prediv: PllPreDiv::DIV32, // 8MHz OSC / 4 = 2 MHz
            mul: PllMul::MUL400,      // 800 MHz
            divp: Some(PllDiv::DIV2), // 400 MHz for System Clock
            divq: Some(PllDiv::DIV8), // 100 MHz for SDMMC
            divr: Some(PllDiv::DIV2), // 400 MHz (not used)
        });

        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,    // 8MHz
            prediv: PllPreDiv::DIV32,  // 8MHz OSC / 4 = 2 MHz
            mul: PllMul::MUL240,       // 480 MHz
            divp: Some(PllDiv::DIV30), // 16 MHz for SPI 1,2,3
            divq: Some(PllDiv::DIV30), // 16 MHz for SPI 4,5, and FDCAN
            divr: Some(PllDiv::DIV5),  // 96 MHz (not used)
        });

        config.rcc.pll3 = Some(Pll {
            source: PllSource::HSI,    // 8MHz
            prediv: PllPreDiv::DIV32,  // 8MHz OSC / 4 = 2 MHz
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
