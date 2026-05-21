pub mod health;
pub mod ingestion;
pub mod processors;

use crate::{errors::SensorError, packets::*};
use libm::pow;

#[derive(Default)]
pub struct SensorBus {
    pub imu: Option<Result<ImuPacket, SensorError>>,
    pub mag: Option<Result<MagPacket, SensorError>>,
    pub baro: Option<Result<BaroPacket, SensorError>>,
    pub pitot: Option<Result<PitotPacket, SensorError>>,
    pub range: Option<Result<RangePacket, SensorError>>,
    pub gnss: Option<Result<GNSSPacket, SensorError>>,
    pub battery: Option<Result<BatteryPacket, SensorError>>,
    pub rc: Option<Result<RcPacket, SensorError>>,
    pub attitude: Option<Result<AttitudePacket, SensorError>>,
}

impl SensorBus {
    pub fn clear(&mut self) {
        self.imu = None;
        self.mag = None;
        self.baro = None;
        self.pitot = None;
        self.range = None;
        self.gnss = None;
        self.battery = None;
        self.rc = None;
        self.attitude = None;
    }
}

#[derive(Default)]
pub struct ProcessedSensors {
    pub imu: Option<ImuPacket>,
    pub mag: Option<MagPacket>,
    pub baro: Option<BaroPacket>,
    pub pitot: Option<PitotPacket>,
    pub range: Option<RangePacket>,
    pub gnss: Option<GNSSPacket>,
    pub battery: Option<BatteryPacket>,
    pub rc: Option<RcPacket>,
    pub attitude: Option<AttitudePacket>,
}

impl ProcessedSensors {
    pub fn clear(&mut self) {
        self.imu = None;
        self.mag = None;
        self.baro = None;
        self.pitot = None;
        self.range = None;
        self.gnss = None;
        self.battery = None;
        self.rc = None;
        self.attitude = None;
    }

    pub fn air_density(&self) -> f64 {
        self.baro
            .map(|baro| 1.225 * pow(baro.pressure as f64 / 101_325.0, 0.809736894596450))
            .unwrap_or(1.225)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_resources_default_to_empty() {
        let raw = SensorBus::default();
        let processed = ProcessedSensors::default();

        assert!(raw.imu.is_none());
        assert!(raw.rc.is_none());
        assert!(processed.imu.is_none());
        assert!(processed.rc.is_none());
    }

    #[test]
    fn processed_sensors_reports_rosflight_air_density_from_baro_pressure() {
        let mut processed = ProcessedSensors::default();
        assert_eq!(processed.air_density(), 1.225);

        processed.baro = Some(BaroPacket {
            pressure: 80_000.0,
            ..Default::default()
        });

        let expected = 1.225 * libm::pow(80_000.0 / 101_325.0, 0.809736894596450);
        assert!((processed.air_density() - expected).abs() < 1e-12);
    }
}
