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

pub trait SensorProcessorAccess<R: FlightFloat> {
    type Imu: SensorPacketProcessor<ImuPacket<R>>;
    type Mag: SensorPacketProcessor<MagPacket>;
    type Baro: SensorPacketProcessor<BaroPacket>;
    type Pitot: SensorPacketProcessor<PitotPacket>;
    type Range: SensorPacketProcessor<RangePacket>;
    type Gnss: SensorPacketProcessor<GNSSPacket>;
    type Battery: SensorPacketProcessor<BatteryPacket>;
    type Rc: SensorPacketProcessor<RcPacket>;
    type Attitude: SensorPacketProcessor<AttitudePacket>;

    fn imu(&mut self) -> &mut Self::Imu;
    fn mag(&mut self) -> &mut Self::Mag;
    fn baro(&mut self) -> &mut Self::Baro;
    fn pitot(&mut self) -> &mut Self::Pitot;
    fn range(&mut self) -> &mut Self::Range;
    fn gnss(&mut self) -> &mut Self::Gnss;
    fn battery(&mut self) -> &mut Self::Battery;
    fn rc(&mut self) -> &mut Self::Rc;
    fn attitude(&mut self) -> &mut Self::Attitude;
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
> SensorProcessorAccess<R>
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
    type Imu = ImuProc;
    type Mag = MagProc;
    type Baro = BaroProc;
    type Pitot = PitotProc;
    type Range = RangeProc;
    type Gnss = GnssProc;
    type Battery = BatteryProc;
    type Rc = RcProc;
    type Attitude = AttitudeProc;

    fn imu(&mut self) -> &mut Self::Imu {
        &mut self.imu
    }

    fn mag(&mut self) -> &mut Self::Mag {
        &mut self.mag
    }

    fn baro(&mut self) -> &mut Self::Baro {
        &mut self.baro
    }

    fn pitot(&mut self) -> &mut Self::Pitot {
        &mut self.pitot
    }

    fn range(&mut self) -> &mut Self::Range {
        &mut self.range
    }

    fn gnss(&mut self) -> &mut Self::Gnss {
        &mut self.gnss
    }

    fn battery(&mut self) -> &mut Self::Battery {
        &mut self.battery
    }

    fn rc(&mut self) -> &mut Self::Rc {
        &mut self.rc
    }

    fn attitude(&mut self) -> &mut Self::Attitude {
        &mut self.attitude
    }
}

pub struct SensorIngestionCtx<'a, R: FlightFloat, Processors = SensorProcessorSet<R>> {
    pub raw: &'a mut SensorBus<R>,
    pub processed: &'a mut ProcessedSensors<R>,
    pub processors: &'a mut Processors,
    pub flags: &'a mut CalibrationFlags,
    pub params: &'a mut Params,
}

pub fn process_sensor_bus<R, Processors>(ctx: SensorIngestionCtx<'_, R, Processors>)
where
    R: FlightFloat,
    Processors: SensorProcessorAccess<R>,
{
    let SensorIngestionCtx {
        raw,
        processed,
        processors,
        flags,
        params,
    } = ctx;

    if raw.imu.is_some() {
        processed.imu = processors.imu().process(&mut raw.imu, flags, params);
    } else {
        processed.imu = None;
    }
    if raw.mag.is_some() {
        processed.mag = processors.mag().process(&mut raw.mag, flags, params);
    } else {
        processed.mag = None;
    }
    if raw.baro.is_some() {
        processed.baro = processors.baro().process(&mut raw.baro, flags, params);
    } else {
        processed.baro = None;
    }
    if raw.pitot.is_some() {
        processed.pitot = processors.pitot().process(&mut raw.pitot, flags, params);
    } else {
        processed.pitot = None;
    }
    if raw.range.is_some() {
        processed.range = processors.range().process(&mut raw.range, flags, params);
    } else {
        processed.range = None;
    }
    if raw.gnss.is_some() {
        processed.gnss = processors.gnss().process(&mut raw.gnss, flags, params);
    } else {
        processed.gnss = None;
    }
    if raw.battery.is_some() {
        processed.battery = processors
            .battery()
            .process(&mut raw.battery, flags, params);
    }
    if raw.rc.is_some() {
        processed.rc = processors.rc().process(&mut raw.rc, flags, params);
    } else {
        processed.rc = None;
    }
    if raw.attitude.is_some() {
        processed.attitude = processors
            .attitude()
            .process(&mut raw.attitude, flags, params);
    } else {
        processed.attitude = None;
    }
}

pub fn process_imu_sensor<R, Processors>(ctx: SensorIngestionCtx<'_, R, Processors>)
where
    R: FlightFloat,
    Processors: SensorProcessorAccess<R>,
{
    let SensorIngestionCtx {
        raw,
        processed,
        processors,
        flags,
        params,
    } = ctx;

    if raw.imu.is_some() {
        processed.imu = processors.imu().process(&mut raw.imu, flags, params);
    } else {
        processed.imu = None;
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

        process_sensor_bus(SensorIngestionCtx {
            raw: &mut raw,
            processed: &mut processed,
            processors: &mut processors,
            flags: &mut flags,
            params: &mut params,
        });

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

        process_sensor_bus(SensorIngestionCtx {
            raw: &mut raw,
            processed: &mut processed,
            processors: &mut processors,
            flags: &mut flags,
            params: &mut params,
        });

        assert!(processed.imu.is_none());
        assert!(processed.rc.is_none());
    }

    #[test]
    fn process_imu_sensor_leaves_service_sensor_state_intact() {
        let mut raw = SensorBus::<f64>::default();
        let mut processed = ProcessedSensors::<f64> {
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

        raw.imu = Some(Ok(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 200,
                status: 0,
            },
            ..Default::default()
        }));

        process_imu_sensor(SensorIngestionCtx {
            raw: &mut raw,
            processed: &mut processed,
            processors: &mut processors,
            flags: &mut flags,
            params: &mut params,
        });

        assert!(raw.imu.is_none());
        assert_eq!(processed.imu.unwrap().header.timestamp, 200);
        assert_eq!(processed.rc.unwrap().header.timestamp, 100);
    }
}
