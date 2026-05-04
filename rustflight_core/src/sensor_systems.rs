use crate::{
    errors::SensorError,
    hlist::Func,
    packets::*,
    params2::Params,
    sensorprocessors::{
        CalibrationFlags, ImuProcessor, MagProcessor, PassthroughAttitudeProcessor,
        PassthroughBaroProcessor, PassthroughBatteryProcessor, PassthroughGNSSProcessor,
        PassthroughPitotProcessor, PassthroughRangeProcessor, PassthroughRcProcessor,
    },
    sensors::{ProcessedSensors, SensorBus},
};

pub struct SensorProcessorSet<
    ImuProc = ImuProcessor,
    MagProc = MagProcessor,
    BaroProc = PassthroughBaroProcessor,
    PitotProc = PassthroughPitotProcessor,
    RangeProc = PassthroughRangeProcessor,
    GnssProc = PassthroughGNSSProcessor,
    BatteryProc = PassthroughBatteryProcessor,
    RcProc = PassthroughRcProcessor,
    AttitudeProc = PassthroughAttitudeProcessor,
> {
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
    raw: &mut SensorBus,
    processed: &mut ProcessedSensors,
    processors: &mut SensorProcessorSet<
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
    for<'a> ImuProc:
        Func<&'a mut Option<Result<ImuPacket, SensorError>>, Output = Option<ImuPacket>>,
    for<'a> MagProc:
        Func<&'a mut Option<Result<MagPacket, SensorError>>, Output = Option<MagPacket>>,
    for<'a> BaroProc:
        Func<&'a mut Option<Result<BaroPacket, SensorError>>, Output = Option<BaroPacket>>,
    for<'a> PitotProc:
        Func<&'a mut Option<Result<PitotPacket, SensorError>>, Output = Option<PitotPacket>>,
    for<'a> RangeProc:
        Func<&'a mut Option<Result<RangePacket, SensorError>>, Output = Option<RangePacket>>,
    for<'a> GnssProc:
        Func<&'a mut Option<Result<GNSSPacket, SensorError>>, Output = Option<GNSSPacket>>,
    for<'a> BatteryProc:
        Func<&'a mut Option<Result<BatteryPacket, SensorError>>, Output = Option<BatteryPacket>>,
    for<'a> RcProc: Func<&'a mut Option<Result<RcPacket, SensorError>>, Output = Option<RcPacket>>,
    for<'a> AttitudeProc:
        Func<&'a mut Option<Result<AttitudePacket, SensorError>>, Output = Option<AttitudePacket>>,
{
    processed.imu = processors.imu.call(&mut raw.imu, flags, params);
    processed.mag = processors.mag.call(&mut raw.mag, flags, params);
    processed.baro = processors.baro.call(&mut raw.baro, flags, params);
    processed.pitot = processors.pitot.call(&mut raw.pitot, flags, params);
    processed.range = processors.range.call(&mut raw.range, flags, params);
    processed.gnss = processors.gnss.call(&mut raw.gnss, flags, params);
    processed.battery = processors.battery.call(&mut raw.battery, flags, params);
    processed.rc = processors.rc.call(&mut raw.rc, flags, params);
    processed.attitude = processors.attitude.call(&mut raw.attitude, flags, params);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        packets::RosflightPacketHeader,
        sensorprocessors::{PassthroughImuProcessor, PassthroughMagProcessor},
    };

    #[test]
    fn process_sensor_bus_moves_raw_packets_into_named_processed_fields() {
        let mut raw = SensorBus::default();
        let mut processed = ProcessedSensors::default();
        let mut processors = SensorProcessorSet::<
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
}
