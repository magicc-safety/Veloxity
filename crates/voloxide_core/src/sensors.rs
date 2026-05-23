pub mod health;
pub mod ingestion;
pub mod processors;

use crate::{errors::SensorError, math::FlightFloat, packets::*};

#[derive(Default)]
pub struct SensorBus<R: FlightFloat> {
    pub imu: Option<Result<ImuPacket<R>, SensorError>>,
    pub mag: Option<Result<MagPacket, SensorError>>,
    pub baro: Option<Result<BaroPacket, SensorError>>,
    pub pitot: Option<Result<PitotPacket, SensorError>>,
    pub range: Option<Result<RangePacket, SensorError>>,
    pub gnss: Option<Result<GNSSPacket, SensorError>>,
    pub battery: Option<Result<BatteryPacket, SensorError>>,
    pub rc: Option<Result<RcPacket, SensorError>>,
    pub attitude: Option<Result<AttitudePacket, SensorError>>,
}

impl<R: FlightFloat> SensorBus<R> {
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
pub struct ProcessedSensors<R: FlightFloat> {
    pub imu: Option<ImuPacket<R>>,
    pub mag: Option<MagPacket>,
    pub baro: Option<BaroPacket>,
    pub pitot: Option<PitotPacket>,
    pub range: Option<RangePacket>,
    pub gnss: Option<GNSSPacket>,
    pub battery: Option<BatteryPacket>,
    pub rc: Option<RcPacket>,
    pub attitude: Option<AttitudePacket>,
}

impl<R: FlightFloat> ProcessedSensors<R> {
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

    pub fn air_density(&self) -> R {
        self.baro
            .map(|baro| {
                <R as FlightFloat>::from_f32(1.225)
                    * (<R as FlightFloat>::from_f32(baro.pressure)
                        / <R as FlightFloat>::from_f32(101_325.0))
                    .powf(<R as FlightFloat>::from_f64(0.809736894596450))
            })
            .unwrap_or_else(|| <R as FlightFloat>::from_f32(1.225))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_resources_default_to_empty() {
        let raw = SensorBus::<f64>::default();
        let processed = ProcessedSensors::<f64>::default();

        assert!(raw.imu.is_none());
        assert!(raw.rc.is_none());
        assert!(processed.imu.is_none());
        assert!(processed.rc.is_none());
    }

    #[test]
    fn processed_sensors_reports_rosflight_air_density_from_baro_pressure() {
        let mut processed = ProcessedSensors::<f64>::default();
        assert!((processed.air_density() - 1.225).abs() < 1e-6);

        processed.baro = Some(BaroPacket {
            pressure: 80_000.0,
            ..Default::default()
        });

        let expected = 1.225 * (80_000.0_f64 / 101_325.0).powf(0.809736894596450);
        assert!((processed.air_density() - expected).abs() < 1e-6);
    }
}
