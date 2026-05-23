use rp2350_platform::{
    multicore::{CoreAssignment, MulticoreMailboxConfig},
    pio::{PioAllocation, PioBlock, PioPurpose, StateMachine},
    wifi::WifiCommsConfig,
};

pub const MAX_PWM_OUTPUTS: usize = 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pico2WConfig {
    pub cores: CoreAssignment,
    pub mailbox: MulticoreMailboxConfig,
    pub wifi: WifiCommsConfig,
    pub pio_allocations: &'static [PioAllocation],
    pub pinout: Pico2WPinout,
}

impl Default for Pico2WConfig {
    fn default() -> Self {
        Self {
            cores: CoreAssignment::default(),
            mailbox: MulticoreMailboxConfig::default(),
            wifi: WifiCommsConfig::default(),
            pio_allocations: DEFAULT_PIO_ALLOCATIONS,
            pinout: Pico2WPinout::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pico2WPinout {
    pub esc: DshotEscPinout,
    pub imu: ImuSpiPinout,
    pub rc: RcReceiverPinout,
    pub leds: StatusLedPinout,
}

impl Default for Pico2WPinout {
    fn default() -> Self {
        Self {
            esc: DshotEscPinout::default(),
            imu: ImuSpiPinout::default(),
            rc: RcReceiverPinout::default(),
            leds: StatusLedPinout::default(),
        }
    }
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
            telemetry_gpio: Some(6),
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
    pub bmp_cs_gpio: u8,
    pub data_ready_gpio: Option<u8>,
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
            bmp_cs_gpio: 14,
            data_ready_gpio: None,
            sensor: ImuSensorKind::Mpu6500Bmp280,
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
pub struct StatusLedPinout {
    pub flight_status_gpio: u8,
    pub comms_status_gpio: u8,
    pub fault_status_gpio: u8,
    pub addressable_gpio: Option<u8>,
}

impl Default for StatusLedPinout {
    fn default() -> Self {
        Self {
            flight_status_gpio: 16,
            comms_status_gpio: 17,
            fault_status_gpio: 18,
            addressable_gpio: Some(19),
        }
    }
}

pub const DEFAULT_PIO_ALLOCATIONS: &[PioAllocation] = &[
    PioAllocation::new(PioBlock::Pio0, StateMachine::Sm0, PioPurpose::Cyw43Wifi),
    PioAllocation::new(PioBlock::Pio1, StateMachine::Sm0, PioPurpose::MotorOutput),
    PioAllocation::new(
        PioBlock::Pio1,
        StateMachine::Sm1,
        PioPurpose::MotorTelemetry,
    ),
    PioAllocation::new(PioBlock::Pio2, StateMachine::Sm0, PioPurpose::StatusLed),
];
