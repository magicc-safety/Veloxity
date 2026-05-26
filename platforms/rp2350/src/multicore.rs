#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoreRole {
    FlightControl,
    PeripheralService,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoreAssignment {
    pub core0: CoreRole,
    pub core1: CoreRole,
}

impl Default for CoreAssignment {
    fn default() -> Self {
        Self {
            core0: CoreRole::FlightControl,
            core1: CoreRole::PeripheralService,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MulticoreMailboxConfig {
    pub rx_bytes: usize,
    pub tx_bytes: usize,
}

impl Default for MulticoreMailboxConfig {
    fn default() -> Self {
        Self {
            rx_bytes: 4096,
            tx_bytes: 4096,
        }
    }
}
