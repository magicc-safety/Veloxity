use crate::{
    multicore::{Core0FlightConfig, Core1ServiceConfig, CoreAssignment, MulticoreMailboxConfig},
    pio::{PioAllocation, PioBlock, PioPurpose, StateMachine},
};

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
pub enum HardwareI2cBus {
    I2c0,
    I2c1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpsSerialBus {
    PioUart,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuSensorKind {
    Ism330dhcx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PressureSensorKind {
    Ms5611,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RcReceiverProtocol {
    Crsf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlatformConfig {
    pub cores: CoreAssignment,
    pub core0: Core0FlightConfig,
    pub core1: Core1ServiceConfig,
    pub mailbox: MulticoreMailboxConfig,
    pub pio_allocations: &'static [PioAllocation],
}

impl Default for PlatformConfig {
    fn default() -> Self {
        Self {
            cores: CoreAssignment::default(),
            core0: Core0FlightConfig::default(),
            core1: Core1ServiceConfig::default(),
            mailbox: MulticoreMailboxConfig::default(),
            pio_allocations: DEFAULT_PIO_ALLOCATIONS,
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
