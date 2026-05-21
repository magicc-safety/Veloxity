#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WifiSecurity {
    Open,
    Wpa2,
    Wpa3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WifiNetworkConfig {
    pub ssid: &'static str,
    pub passphrase: &'static str,
    pub security: WifiSecurity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UdpMavlinkConfig {
    pub bind_port: u16,
    pub peer_port: u16,
}

impl Default for UdpMavlinkConfig {
    fn default() -> Self {
        Self {
            bind_port: 14550,
            peer_port: 14550,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WifiCommsConfig {
    pub udp: UdpMavlinkConfig,
    pub rx_bytes: usize,
    pub tx_bytes: usize,
}

impl Default for WifiCommsConfig {
    fn default() -> Self {
        Self {
            udp: UdpMavlinkConfig::default(),
            rx_bytes: 4096,
            tx_bytes: 4096,
        }
    }
}
