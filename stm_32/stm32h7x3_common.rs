use core::default::Default;
use core::option::Option::Some;
use embassy_stm32::{Config, rcc};

//use defmt::*;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::InterruptExecutor;
use embassy_stm32::Peripherals as EMBASSY_Peripherals;
use embassy_stm32::bind_interrupts;
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
//use {defmt_rtt as _, panic_probe as _};


// pub struct UartResources {
//     pub uart: usart::BufferedUart<'static>,
// }

// All STM32 SPI's
#[allow(dead_code)]
static SPI1_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async>>> =
    StaticCell::new();
#[allow(dead_code)]
static SPI2_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async>>> =
    StaticCell::new();
#[allow(dead_code)]
static SPI3_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async>>> =
    StaticCell::new();
#[allow(dead_code)]
static SPI4_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async>>> =
    StaticCell::new();
#[allow(dead_code)]
static SPI5_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async>>> =
    StaticCell::new();
#[allow(dead_code)]
static SPI6_BUS: StaticCell<Mutex<CriticalSectionRawMutex, spi::Spi<'static, Async>>> =
    StaticCell::new();

// All STM32 I2C's
#[allow(dead_code)]
static I2C1_BUS: StaticCell<Mutex<CriticalSectionRawMutex, i2c::I2c<'static, Async>>> =
    StaticCell::new();
#[allow(dead_code)]
static I2C2_BUS: StaticCell<Mutex<CriticalSectionRawMutex, i2c::I2c<'static, Async>>> =
    StaticCell::new();
#[allow(dead_code)]
static I2C3_BUS: StaticCell<Mutex<CriticalSectionRawMutex, i2c::I2c<'static, Async>>> =
    StaticCell::new();
#[allow(dead_code)]
static I2C4_BUS: StaticCell<Mutex<CriticalSectionRawMutex, i2c::I2c<'static, Async>>> =
    StaticCell::new();

// All I2C Interrupts
bind_interrupts!(struct IrqsI2c1 {
    I2C1_EV => i2c::EventInterruptHandler<EMBASSY_peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<EMBASSY_peripherals::I2C1>;
});
bind_interrupts!(struct IrqsI2c2 {
    I2C2_EV => i2c::EventInterruptHandler<EMBASSY_peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<EMBASSY_peripherals::I2C2>;
});
bind_interrupts!(struct IrqsI2c3 {
    I2C3_EV => i2c::EventInterruptHandler<EMBASSY_peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<EMBASSY_peripherals::I2C3>;
});
bind_interrupts!(struct IrqsI2c4 {
    I2C4_EV => i2c::EventInterruptHandler<EMBASSY_peripherals::I2C4>;
    I2C4_ER => i2c::ErrorInterruptHandler<EMBASSY_peripherals::I2C4>;
});

// All USART Interrupts
bind_interrupts!(struct Usart1Irqs {
    USART1 => usart::InterruptHandler<EMBASSY_peripherals::USART1>;
});

bind_interrupts!(struct Usart2Irqs {
    USART2 => usart::InterruptHandler<EMBASSY_peripherals::USART2>;
});

bind_interrupts!(struct Usart3Irqs {
    USART3 => usart::InterruptHandler<EMBASSY_peripherals::USART3>;
});

bind_interrupts!(struct Uart6Irqs {
    USART6 => usart::InterruptHandler<EMBASSY_peripherals::USART6>;
});

// All UART Interrupts
bind_interrupts!(struct Uart4Irqs {
    UART4 => usart::InterruptHandler<EMBASSY_peripherals::UART4>;
});
bind_interrupts!(struct Uart5Irqs {
    UART5 => usart::InterruptHandler<EMBASSY_peripherals::UART5>;
});
bind_interrupts!(struct Uart7Irqs {
    UART7 => usart::InterruptHandler<EMBASSY_peripherals::UART7>;
});
bind_interrupts!(struct Uart8Irqs {
    UART8 => usart::InterruptHandler<EMBASSY_peripherals::UART8>;
});

// SDMMC 1 Interrupts
bind_interrupts!(struct Sdmmc1Irqs {
    SDMMC1 => sdmmc::InterruptHandler<EMBASSY_peripherals::SDMMC1>;
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
