use rp2350_platform::{
    multicore::{CoreAssignment, MulticoreMailboxConfig},
    pio::{PioAllocation, PioBlock, PioPurpose, StateMachine},
};

pub const MAX_PWM_OUTPUTS: usize = 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pico2WConfig {
    pub cores: CoreAssignment,
    pub mailbox: MulticoreMailboxConfig,
    pub pio_allocations: &'static [PioAllocation],
    pub pinout: Pico2WPinout,
}

impl Default for Pico2WConfig {
    fn default() -> Self {
        Self {
            cores: CoreAssignment::default(),
            mailbox: MulticoreMailboxConfig::default(),
            pio_allocations: DEFAULT_PIO_ALLOCATIONS,
            pinout: Pico2WPinout::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pico2WPinout {
    pub companion: CompanionUartPinout,
    pub gps: GpsPinout,
    pub esc: DshotEscPinout,
    pub imu: ImuSpiPinout,
    pub rc: RcReceiverPinout,
    pub slow_i2c: SlowI2cPinout,
    pub leds: StatusLedPinout,
}

impl Default for Pico2WPinout {
    fn default() -> Self {
        Self {
            companion: CompanionUartPinout::default(),
            gps: GpsPinout::default(),
            esc: DshotEscPinout::default(),
            imu: ImuSpiPinout::default(),
            rc: RcReceiverPinout::default(),
            slow_i2c: SlowI2cPinout::default(),
            leds: StatusLedPinout::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompanionUartPinout {
    pub uart_bus: HardwareUartBus,
    pub tx_gpio: u8,
    pub rx_gpio: u8,
    pub baudrate: u32,
}

impl Default for CompanionUartPinout {
    fn default() -> Self {
        Self {
            uart_bus: HardwareUartBus::Uart0,
            tx_gpio: 0,
            rx_gpio: 1,
            baudrate: 2_000_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsPinout {
    pub serial_bus: GpsSerialBus,
    pub tx_gpio: u8,
    pub rx_gpio: u8,
    pub pps_gpio: Option<u8>,
    pub magnetometer_data_ready_gpio: Option<u8>,
}

impl Default for GpsPinout {
    fn default() -> Self {
        Self {
            serial_bus: GpsSerialBus::PioUart,
            tx_gpio: 7,
            rx_gpio: 6,
            pps_gpio: Some(16),
            magnetometer_data_ready_gpio: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpsSerialBus {
    PioUart,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DshotEscPinout {
    pub motor_signal_gpios: [u8; 4],
    pub telemetry_gpio: Option<u8>,
}

impl Default for DshotEscPinout {
    fn default() -> Self {
        Self {
            motor_signal_gpios: [2, 3, 4, 5],
            telemetry_gpio: Some(17),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImuSpiPinout {
    pub spi_bus: HardwareSpiBus,
    pub sck_gpio: u8,
    pub mosi_gpio: u8,
    pub miso_gpio: u8,
    pub cs_gpio: u8,
    pub bmp_cs_gpio: Option<u8>,
    pub data_ready_gpio: Option<u8>,
    pub aux_interrupt_gpio: Option<u8>,
    pub sensor: ImuSensorKind,
}

impl Default for ImuSpiPinout {
    fn default() -> Self {
        Self {
            spi_bus: HardwareSpiBus::Spi1,
            sck_gpio: 10,
            mosi_gpio: 11,
            miso_gpio: 12,
            cs_gpio: 13,
            bmp_cs_gpio: None,
            data_ready_gpio: Some(14),
            aux_interrupt_gpio: Some(15),
            sensor: ImuSensorKind::Ism330dhcx,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareSpiBus {
    Spi0,
    Spi1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareUartBus {
    Uart0,
    Uart1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuSensorKind {
    Mpu6500Bmp280,
    Ism330dhcx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcReceiverPinout {
    pub protocol: RcReceiverProtocol,
    pub uart_bus: HardwareUartBus,
    pub tx_gpio: u8,
    pub rx_gpio: u8,
    pub baudrate: u32,
    pub pio_fallback_gpio: Option<u8>,
}

impl Default for RcReceiverPinout {
    fn default() -> Self {
        Self {
            protocol: RcReceiverProtocol::Crsf,
            uart_bus: HardwareUartBus::Uart1,
            tx_gpio: 8,
            rx_gpio: 9,
            baudrate: 420_000,
            pio_fallback_gpio: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RcReceiverProtocol {
    Crsf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlowI2cPinout {
    pub bus: HardwareI2cBus,
    pub sda_gpio: u8,
    pub scl_gpio: u8,
}

impl Default for SlowI2cPinout {
    fn default() -> Self {
        Self {
            bus: HardwareI2cBus::I2c0,
            sda_gpio: 20,
            scl_gpio: 21,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareI2cBus {
    I2c0,
    I2c1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusLedPinout {
    pub flight_status_gpio: Option<u8>,
    pub comms_status_gpio: Option<u8>,
    pub fault_status_gpio: Option<u8>,
    pub addressable_gpio: Option<u8>,
}

impl Default for StatusLedPinout {
    fn default() -> Self {
        Self {
            flight_status_gpio: Some(18),
            comms_status_gpio: None,
            fault_status_gpio: None,
            addressable_gpio: Some(19),
        }
    }
}

pub const DEFAULT_PIO_ALLOCATIONS: &[PioAllocation] = &[
    PioAllocation::new(PioBlock::Pio0, StateMachine::Sm0, PioPurpose::Reserved),
    PioAllocation::new(PioBlock::Pio1, StateMachine::Sm0, PioPurpose::MotorOutput),
    PioAllocation::new(
        PioBlock::Pio1,
        StateMachine::Sm1,
        PioPurpose::MotorTelemetry,
    ),
    PioAllocation::new(PioBlock::Pio2, StateMachine::Sm0, PioPurpose::StatusLed),
];
