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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Core0FlightConfig {
    pub control_loop_hz: u16,
    pub telemetry_streams_per_service_phase: usize,
}

impl Default for Core0FlightConfig {
    fn default() -> Self {
        Self {
            control_loop_hz: 1_500,
            telemetry_streams_per_service_phase: 2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Core1ServiceConfig {
    pub heartbeat: bool,
    pub mavlink_tx: bool,
    pub mavlink_rx: bool,
    pub crsf_rx: bool,
    pub gps: bool,
    pub imu: bool,
    pub pressure: bool,
}

impl Default for Core1ServiceConfig {
    fn default() -> Self {
        Self {
            heartbeat: true,
            mavlink_tx: true,
            mavlink_rx: true,
            crsf_rx: true,
            gps: true,
            imu: true,
            pressure: true,
        }
    }
}
