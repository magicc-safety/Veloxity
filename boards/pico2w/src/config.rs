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
}

impl Default for Pico2WConfig {
    fn default() -> Self {
        Self {
            cores: CoreAssignment::default(),
            mailbox: MulticoreMailboxConfig::default(),
            wifi: WifiCommsConfig::default(),
            pio_allocations: DEFAULT_PIO_ALLOCATIONS,
        }
    }
}

pub const DEFAULT_PIO_ALLOCATIONS: &[PioAllocation] = &[
    PioAllocation::new(PioBlock::Pio0, StateMachine::Sm0, PioPurpose::Cyw43Wifi),
    PioAllocation::new(PioBlock::Pio0, StateMachine::Sm1, PioPurpose::MotorOutput),
    PioAllocation::new(PioBlock::Pio0, StateMachine::Sm2, PioPurpose::MotorOutput),
    PioAllocation::new(PioBlock::Pio0, StateMachine::Sm3, PioPurpose::ServoOutput),
    PioAllocation::new(PioBlock::Pio1, StateMachine::Sm0, PioPurpose::SbusInput),
    PioAllocation::new(PioBlock::Pio1, StateMachine::Sm1, PioPurpose::SpiSensor),
    PioAllocation::new(PioBlock::Pio1, StateMachine::Sm2, PioPurpose::I2cSensor),
];
