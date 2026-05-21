#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PioBlock {
    Pio0,
    Pio1,
    Pio2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateMachine {
    Sm0,
    Sm1,
    Sm2,
    Sm3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PioPurpose {
    MotorOutput,
    ServoOutput,
    SbusInput,
    I2cSensor,
    SpiSensor,
    Cyw43Wifi,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PioAllocation {
    pub block: PioBlock,
    pub state_machine: StateMachine,
    pub purpose: PioPurpose,
}

impl PioAllocation {
    pub const fn new(block: PioBlock, state_machine: StateMachine, purpose: PioPurpose) -> Self {
        Self {
            block,
            state_machine,
            purpose,
        }
    }
}
