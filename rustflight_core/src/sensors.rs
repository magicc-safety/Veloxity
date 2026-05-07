use crate::{errors::SensorError, hlist::HListGet, packets::*};

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
}

pub fn processed_sensors_from_hlist<
    S,
    ImuIdx,
    MagIdx,
    BaroIdx,
    PitotIdx,
    RangeIdx,
    GnssIdx,
    BatteryIdx,
    AttitudeIdx,
    RcIdx,
>(
    processed_sensors: &S,
) -> ProcessedSensors
where
    S: HListGet<Option<ImuPacket>, ImuIdx>
        + HListGet<Option<MagPacket>, MagIdx>
        + HListGet<Option<BaroPacket>, BaroIdx>
        + HListGet<Option<PitotPacket>, PitotIdx>
        + HListGet<Option<RangePacket>, RangeIdx>
        + HListGet<Option<GNSSPacket>, GnssIdx>
        + HListGet<Option<BatteryPacket>, BatteryIdx>
        + HListGet<Option<AttitudePacket>, AttitudeIdx>
        + HListGet<Option<RcPacket>, RcIdx>,
{
    ProcessedSensors {
        imu: *processed_sensors.get(),
        mag: *processed_sensors.get(),
        baro: *processed_sensors.get(),
        pitot: *processed_sensors.get(),
        range: *processed_sensors.get(),
        gnss: *processed_sensors.get(),
        battery: *processed_sensors.get(),
        rc: *processed_sensors.get(),
        attitude: *processed_sensors.get(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hlist::{HCons, HNil, Here, There};

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
    fn processed_sensors_from_hlist_maps_legacy_slots_to_named_fields() {
        type I0 = Here;
        type I1 = There<I0>;
        type I2 = There<I1>;
        type I3 = There<I2>;
        type I4 = There<I3>;
        type I5 = There<I4>;
        type I6 = There<I5>;
        type I7 = There<I6>;
        type I8 = There<I7>;

        let imu = ImuPacket {
            seq: 7,
            ..Default::default()
        };
        let mag = MagPacket {
            flux: [1.0, 2.0, 3.0],
            ..Default::default()
        };
        let rc = RcPacket {
            n_chan: 4,
            ..Default::default()
        };
        let processed_hlist = HCons(
            Some(imu),
            HCons(
                Some(mag),
                HCons(
                    None::<BaroPacket>,
                    HCons(
                        None::<PitotPacket>,
                        HCons(
                            None::<RangePacket>,
                            HCons(
                                None::<GNSSPacket>,
                                HCons(
                                    None::<BatteryPacket>,
                                    HCons(None::<AttitudePacket>, HCons(Some(rc), HNil)),
                                ),
                            ),
                        ),
                    ),
                ),
            ),
        );

        let processed = processed_sensors_from_hlist::<
            _,
            I0,
            I1,
            I2,
            I3,
            I4,
            I5,
            I6,
            I7,
            I8,
        >(&processed_hlist);

        assert_eq!(processed.imu.unwrap().seq, 7);
        assert_eq!(processed.mag.unwrap().flux, [1.0, 2.0, 3.0]);
        assert_eq!(processed.rc.unwrap().n_chan, 4);
        assert!(processed.baro.is_none());
        assert!(processed.gnss.is_none());
    }
}
