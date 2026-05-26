use crate::{
    math::FlightFloat,
    packets::*,
    params::Params,
    sensors::processors::{
        BaroProcessor, BatteryProcessor, CalibrationFlags, ImuProcessor, MagProcessor,
        PassthroughAttitudeProcessor, PassthroughGNSSProcessor, PassthroughRangeProcessor,
        PassthroughRcProcessor, PitotProcessor, SensorPacketProcessor,
    },
    sensors::{ProcessedSensors, SensorBus},
};

pub struct SensorProcessorSet<
    R: FlightFloat,
    ImuProc = ImuProcessor<R>,
    MagProc = MagProcessor,
    BaroProc = BaroProcessor,
    PitotProc = PitotProcessor,
    RangeProc = PassthroughRangeProcessor,
    GnssProc = PassthroughGNSSProcessor,
    BatteryProc = BatteryProcessor,
    RcProc = PassthroughRcProcessor,
    AttitudeProc = PassthroughAttitudeProcessor,
> {
    _real: core::marker::PhantomData<R>,
    pub imu: ImuProc,
    pub mag: MagProc,
    pub baro: BaroProc,
    pub pitot: PitotProc,
    pub range: RangeProc,
    pub gnss: GnssProc,
    pub battery: BatteryProc,
    pub rc: RcProc,
    pub attitude: AttitudeProc,
}

impl<
    R,
    ImuProc,
    MagProc,
    BaroProc,
    PitotProc,
    RangeProc,
    GnssProc,
    BatteryProc,
    RcProc,
    AttitudeProc,
> Default
    for SensorProcessorSet<
        R,
        ImuProc,
        MagProc,
        BaroProc,
        PitotProc,
        RangeProc,
        GnssProc,
        BatteryProc,
        RcProc,
        AttitudeProc,
    >
where
    R: FlightFloat,
    ImuProc: Default,
    MagProc: Default,
    BaroProc: Default,
    PitotProc: Default,
    RangeProc: Default,
    GnssProc: Default,
    BatteryProc: Default,
    RcProc: Default,
    AttitudeProc: Default,
{
    fn default() -> Self {
        Self {
            _real: core::marker::PhantomData,
            imu: ImuProc::default(),
            mag: MagProc::default(),
            baro: BaroProc::default(),
            pitot: PitotProc::default(),
            range: RangeProc::default(),
            gnss: GnssProc::default(),
            battery: BatteryProc::default(),
            rc: RcProc::default(),
            attitude: AttitudeProc::default(),
        }
    }
}

pub fn process_sensor_bus<
    R,
    ImuProc,
    MagProc,
    BaroProc,
    PitotProc,
    RangeProc,
    GnssProc,
    BatteryProc,
    RcProc,
    AttitudeProc,
>(
    raw: &mut SensorBus<R>,
    processed: &mut ProcessedSensors<R>,
    processors: &mut SensorProcessorSet<
        R,
        ImuProc,
        MagProc,
        BaroProc,
        PitotProc,
        RangeProc,
        GnssProc,
        BatteryProc,
        RcProc,
        AttitudeProc,
    >,
    flags: &mut CalibrationFlags,
    params: &mut Params,
) where
    R: FlightFloat,
    ImuProc: SensorPacketProcessor<ImuPacket<R>>,
    MagProc: SensorPacketProcessor<MagPacket>,
    BaroProc: SensorPacketProcessor<BaroPacket>,
    PitotProc: SensorPacketProcessor<PitotPacket>,
    RangeProc: SensorPacketProcessor<RangePacket>,
    GnssProc: SensorPacketProcessor<GNSSPacket>,
    BatteryProc: SensorPacketProcessor<BatteryPacket>,
    RcProc: SensorPacketProcessor<RcPacket>,
    AttitudeProc: SensorPacketProcessor<AttitudePacket>,
{
    if raw.imu.is_some() {
        processed.imu = processors.imu.process(&mut raw.imu, flags, params);
    } else {
        processed.imu = None;
    }
    if raw.mag.is_some() {
        processed.mag = processors.mag.process(&mut raw.mag, flags, params);
    } else {
        processed.mag = None;
    }
    if raw.baro.is_some() {
        processed.baro = processors.baro.process(&mut raw.baro, flags, params);
    } else {
        processed.baro = None;
    }
    if raw.pitot.is_some() {
        processed.pitot = processors.pitot.process(&mut raw.pitot, flags, params);
    } else {
        processed.pitot = None;
    }
    if raw.range.is_some() {
        processed.range = processors.range.process(&mut raw.range, flags, params);
    } else {
        processed.range = None;
    }
    if raw.gnss.is_some() {
        processed.gnss = processors.gnss.process(&mut raw.gnss, flags, params);
    } else {
        processed.gnss = None;
    }
    if raw.battery.is_some() {
        processed.battery = processors.battery.process(&mut raw.battery, flags, params);
    }
    if raw.rc.is_some() {
        processed.rc = processors.rc.process(&mut raw.rc, flags, params);
    } else {
        processed.rc = None;
    }
    if raw.attitude.is_some() {
        processed.attitude = processors
            .attitude
            .process(&mut raw.attitude, flags, params);
    } else {
        processed.attitude = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        packets::RosflightPacketHeader,
        sensors::processors::{
            PassthroughBaroProcessor, PassthroughBatteryProcessor, PassthroughImuProcessor,
            PassthroughMagProcessor, PassthroughPitotProcessor,
        },
    };

    #[test]
    fn process_sensor_bus_moves_raw_packets_into_named_processed_fields() {
        let mut raw = SensorBus::<f64>::default();
        let mut processed = ProcessedSensors::<f64>::default();
        let mut processors = SensorProcessorSet::<
            f64,
            PassthroughImuProcessor,
            PassthroughMagProcessor,
            PassthroughBaroProcessor,
            PassthroughPitotProcessor,
            PassthroughRangeProcessor,
            PassthroughGNSSProcessor,
            PassthroughBatteryProcessor,
            PassthroughRcProcessor,
            PassthroughAttitudeProcessor,
        >::default();
        let mut flags = CalibrationFlags::empty();
        let mut params = Params::new();

        raw.rc = Some(Ok(RcPacket {
            header: RosflightPacketHeader {
                timestamp: 123,
                status: 0,
            },
            n_chan: 1,
            chan: [0.0; RC_PACKET_CHANNELS],
            lol: false,
        }));

        process_sensor_bus(
            &mut raw,
            &mut processed,
            &mut processors,
            &mut flags,
            &mut params,
        );

        assert!(raw.rc.is_none());
        assert_eq!(processed.rc.unwrap().header.timestamp, 123);
    }

    #[test]
    fn process_sensor_bus_clears_stale_one_shot_packets_when_no_raw_packet_arrives() {
        let mut raw = SensorBus::<f64>::default();
        let mut processed = ProcessedSensors::<f64> {
            imu: Some(ImuPacket {
                header: RosflightPacketHeader {
                    timestamp: 100,
                    status: 0,
                },
                ..Default::default()
            }),
            rc: Some(RcPacket {
                header: RosflightPacketHeader {
                    timestamp: 100,
                    status: 0,
                },
                n_chan: 1,
                chan: [0.0; RC_PACKET_CHANNELS],
                lol: false,
            }),
            ..Default::default()
        };
        let mut processors = SensorProcessorSet::<
            f64,
            PassthroughImuProcessor,
            PassthroughMagProcessor,
            PassthroughBaroProcessor,
            PassthroughPitotProcessor,
            PassthroughRangeProcessor,
            PassthroughGNSSProcessor,
            PassthroughBatteryProcessor,
            PassthroughRcProcessor,
            PassthroughAttitudeProcessor,
        >::default();
        let mut flags = CalibrationFlags::empty();
        let mut params = Params::new();

        process_sensor_bus(
            &mut raw,
            &mut processed,
            &mut processors,
            &mut flags,
            &mut params,
        );

        assert!(processed.imu.is_none());
        assert!(processed.rc.is_none());
    }
}
