#[derive(Copy, Clone, PartialEq, Debug)]
pub enum ParamValue {
    Float(f32),
    Int(i32),
    Bool(bool),
}

pub trait Callback {
    type Args<'a>;
}

pub trait Setter {
    fn set<'a>(&mut self, input: <Self as Callback>::Args<'a>)
    where
        Self: Callback;
}

pub struct BaudRate {
    pub value: ParamValue,
}

pub struct SerialDevice {
    pub value: ParamValue,
}

pub struct SystemId {
    pub value: ParamValue,
}

pub struct MaxCommand {
    pub value: ParamValue,
}

pub struct PidRollRateP {
    pub value: ParamValue,
}

pub struct PidRollRateI {
    pub value: ParamValue,
}

pub struct PidRollRateD {
    pub value: ParamValue,
}

pub struct PidPitchRateP {
    pub value: ParamValue,
}

pub struct PidPitchRateI {
    pub value: ParamValue,
}

pub struct PidPitchRateD {
    pub value: ParamValue,
}

pub struct PidYawRateP {
    pub value: ParamValue,
}

pub struct PidYawRateI {
    pub value: ParamValue,
}

pub struct PidYawRateD {
    pub value: ParamValue,
}

pub struct PidRollAngleP {
    pub value: ParamValue,
}

pub struct PidRollAngleI {
    pub value: ParamValue,
}

pub struct PidRollAngleD {
    pub value: ParamValue,
}

pub struct PidPitchAngleP {
    pub value: ParamValue,
}

pub struct PidPitchAngleI {
    pub value: ParamValue,
}

pub struct PidPitchAngleD {
    pub value: ParamValue,
}

pub struct XEqTorque {
    pub value: ParamValue,
}

pub struct YEqTorque {
    pub value: ParamValue,
}

pub struct ZEqTorque {
    pub value: ParamValue,
}

pub struct PidTau {
    pub value: ParamValue,
}

pub struct MotorPwmSendRate {
    pub value: ParamValue,
}

pub struct MotorIdleThrottle {
    pub value: ParamValue,
}

pub struct FailsafeThrottle {
    pub value: ParamValue,
}

pub struct SpinMotorsWhenArmed {
    pub value: ParamValue,
}

pub struct InitTime {
    pub value: ParamValue,
}

pub struct FilterKpAcc {
    pub value: ParamValue,
}

pub struct FilterKi {
    pub value: ParamValue,
}

pub struct FilterKpExt {
    pub value: ParamValue,
}

pub struct FilterAccelMargin {
    pub value: ParamValue,
}

pub struct FilterUseQuadInt {
    pub value: ParamValue,
}

pub struct FilterUseMatExp {
    pub value: ParamValue,
}

pub struct FilterUseAcc {
    pub value: ParamValue,
}

pub struct CalibrateGyroOnArm {
    pub value: ParamValue,
}

pub struct GyroXyAlpha {
    pub value: ParamValue,
}

pub struct GyroZAlpha {
    pub value: ParamValue,
}

pub struct AccAlpha {
    pub value: ParamValue,
}

pub struct GyroXBias {
    pub value: ParamValue,
}

pub struct GyroYBias {
    pub value: ParamValue,
}

pub struct GyroZBias {
    pub value: ParamValue,
}

pub struct AccXBias {
    pub value: ParamValue,
}

pub struct AccYBias {
    pub value: ParamValue,
}

pub struct AccZBias {
    pub value: ParamValue,
}

pub struct AccXTempComp {
    pub value: ParamValue,
}

pub struct AccYTempComp {
    pub value: ParamValue,
}

pub struct AccZTempComp {
    pub value: ParamValue,
}

pub struct MagA11Comp {
    pub value: ParamValue,
}

pub struct MagA12Comp {
    pub value: ParamValue,
}

pub struct MagA13Comp {
    pub value: ParamValue,
}

pub struct MagA21Comp {
    pub value: ParamValue,
}

pub struct MagA22Comp {
    pub value: ParamValue,
}

pub struct MagA23Comp {
    pub value: ParamValue,
}

pub struct MagA31Comp {
    pub value: ParamValue,
}

pub struct MagA32Comp {
    pub value: ParamValue,
}

pub struct MagA33Comp {
    pub value: ParamValue,
}

pub struct MagXBias {
    pub value: ParamValue,
}

pub struct MagYBias {
    pub value: ParamValue,
}

pub struct MagZBias {
    pub value: ParamValue,
}

pub struct BaroBias {
    pub value: ParamValue,
}

pub struct GroundLevel {
    pub value: ParamValue,
}

pub struct DiffPressBias {
    pub value: ParamValue,
}

pub struct RcType {
    pub value: ParamValue,
}

pub struct RcXChannel {
    pub value: ParamValue,
}

pub struct RcYChannel {
    pub value: ParamValue,
}

pub struct RcZChannel {
    pub value: ParamValue,
}

pub struct RcFChannel {
    pub value: ParamValue,
}

pub struct RcAttitudeOverrideChannel {
    pub value: ParamValue,
}

pub struct RcThrottleOverrideChannel {
    pub value: ParamValue,
}

pub struct RcAttControlTypeChannel {
    pub value: ParamValue,
}

pub struct RcArmChannel {
    pub value: ParamValue,
}

pub struct RcNumChannels {
    pub value: ParamValue,
}

pub struct RcSwitch5Direction {
    pub value: ParamValue,
}

pub struct RcSwitch6Direction {
    pub value: ParamValue,
}

pub struct RcSwitch7Direction {
    pub value: ParamValue,
}

pub struct RcSwitch8Direction {
    pub value: ParamValue,
}

pub struct RcOverrideDeviation {
    pub value: ParamValue,
}

pub struct OverrideLagTime {
    pub value: ParamValue,
}

pub struct RcOverrideTakeMinThrottle {
    pub value: ParamValue,
}

pub struct RcAttitudeMode {
    pub value: ParamValue,
}

pub struct RcMaxRoll {
    pub value: ParamValue,
}

pub struct RcMaxPitch {
    pub value: ParamValue,
}

pub struct RcMaxRollRate {
    pub value: ParamValue,
}

pub struct RcMaxPitchRate {
    pub value: ParamValue,
}

pub struct RcMaxYawRate {
    pub value: ParamValue,
}

pub struct Mixer {
    pub value: ParamValue,
}

pub struct FixedWing {
    pub value: ParamValue,
}

pub struct ElevatorReverse {
    pub value: ParamValue,
}

pub struct AileronReverse {
    pub value: ParamValue,
}

pub struct RudderReverse {
    pub value: ParamValue,
}

pub struct FcRoll {
    pub value: ParamValue,
}

pub struct FcPitch {
    pub value: ParamValue,
}

pub struct FcYaw {
    pub value: ParamValue,
}

pub struct ArmThreshold {
    pub value: ParamValue,
}

pub struct OffboardTimeout {
    pub value: ParamValue,
}

pub struct BatteryVoltageMultiplier {
    pub value: ParamValue,
}

pub struct BatteryCurrentMultiplier {
    pub value: ParamValue,
}

pub struct BatteryVoltageAlpha {
    pub value: ParamValue,
}

pub struct BatteryCurrentAlpha {
    pub value: ParamValue,
}

macro_rules! impl_callback {
    ($($type:ty),*) => {
        $(
            impl Callback for $type {
                type Args<'a> = ParamValue;
            }
        )*
    };
}

impl_callback!(
    // BaudRate,
    SerialDevice,
    SystemId,
    MaxCommand,
    PidRollRateP,
    PidRollRateI,
    PidRollRateD,
    PidPitchRateP,
    PidPitchRateI,
    PidPitchRateD,
    PidYawRateP,
    PidYawRateI,
    PidYawRateD,
    PidRollAngleP,
    PidRollAngleI,
    PidRollAngleD,
    PidPitchAngleP,
    PidPitchAngleI,
    PidPitchAngleD,
    XEqTorque,
    YEqTorque,
    ZEqTorque,
    PidTau,
    MotorPwmSendRate,
    MotorIdleThrottle,
    FailsafeThrottle,
    SpinMotorsWhenArmed,
    InitTime,
    FilterKpAcc,
    FilterKi,
    FilterKpExt,
    FilterAccelMargin,
    FilterUseQuadInt,
    FilterUseMatExp,
    FilterUseAcc,
    CalibrateGyroOnArm,
    GyroXyAlpha,
    GyroZAlpha,
    AccAlpha,
    GyroXBias,
    GyroYBias,
    GyroZBias,
    AccXBias,
    AccYBias,
    AccZBias,
    AccXTempComp,
    AccYTempComp,
    AccZTempComp,
    MagA11Comp,
    MagA12Comp,
    MagA13Comp,
    MagA21Comp,
    MagA22Comp,
    MagA23Comp,
    MagA31Comp,
    MagA32Comp,
    MagA33Comp,
    MagXBias,
    MagYBias,
    MagZBias,
    BaroBias,
    GroundLevel,
    DiffPressBias,
    RcType,
    RcXChannel,
    RcYChannel,
    RcZChannel,
    RcFChannel,
    RcAttitudeOverrideChannel,
    RcThrottleOverrideChannel,
    RcAttControlTypeChannel,
    RcArmChannel,
    RcNumChannels,
    RcSwitch5Direction,
    RcSwitch6Direction,
    RcSwitch7Direction,
    RcSwitch8Direction,
    RcOverrideDeviation,
    OverrideLagTime,
    RcOverrideTakeMinThrottle,
    RcAttitudeMode,
    RcMaxRoll,
    RcMaxPitch,
    RcMaxRollRate,
    RcMaxPitchRate,
    RcMaxYawRate,
    Mixer,
    FixedWing,
    ElevatorReverse,
    AileronReverse,
    RudderReverse,
    FcRoll,
    FcPitch,
    FcYaw,
    ArmThreshold,
    OffboardTimeout,
    BatteryVoltageMultiplier,
    BatteryCurrentMultiplier,
    BatteryVoltageAlpha,
    BatteryCurrentAlpha
);

impl Callback for BaudRate {
    type Args<'a> = (&'a mut TestStruct, ParamValue);
}

macro_rules! impl_setter {
    ($($type:ty),*) => {
        $(
            impl Setter for $type {
                fn set<'a>(&mut self, input: <Self as Callback>::Args<'a>) {
                    let data = input;
                    self.value = data;
                }
            }
        )*
    };
}

impl_setter!(
    // BaudRate,
    SerialDevice,
    SystemId,
    MaxCommand,
    PidRollRateP,
    PidRollRateI,
    PidRollRateD,
    PidPitchRateP,
    PidPitchRateI,
    PidPitchRateD,
    PidYawRateP,
    PidYawRateI,
    PidYawRateD,
    PidRollAngleP,
    PidRollAngleI,
    PidRollAngleD,
    PidPitchAngleP,
    PidPitchAngleI,
    PidPitchAngleD,
    XEqTorque,
    YEqTorque,
    ZEqTorque,
    PidTau,
    MotorPwmSendRate,
    MotorIdleThrottle,
    FailsafeThrottle,
    SpinMotorsWhenArmed,
    InitTime,
    FilterKpAcc,
    FilterKi,
    FilterKpExt,
    FilterAccelMargin,
    FilterUseQuadInt,
    FilterUseMatExp,
    FilterUseAcc,
    CalibrateGyroOnArm,
    GyroXyAlpha,
    GyroZAlpha,
    AccAlpha,
    GyroXBias,
    GyroYBias,
    GyroZBias,
    AccXBias,
    AccYBias,
    AccZBias,
    AccXTempComp,
    AccYTempComp,
    AccZTempComp,
    MagA11Comp,
    MagA12Comp,
    MagA13Comp,
    MagA21Comp,
    MagA22Comp,
    MagA23Comp,
    MagA31Comp,
    MagA32Comp,
    MagA33Comp,
    MagXBias,
    MagYBias,
    MagZBias,
    BaroBias,
    GroundLevel,
    DiffPressBias,
    RcType,
    RcXChannel,
    RcYChannel,
    RcZChannel,
    RcFChannel,
    RcAttitudeOverrideChannel,
    RcThrottleOverrideChannel,
    RcAttControlTypeChannel,
    RcArmChannel,
    RcNumChannels,
    RcSwitch5Direction,
    RcSwitch6Direction,
    RcSwitch7Direction,
    RcSwitch8Direction,
    RcOverrideDeviation,
    OverrideLagTime,
    RcOverrideTakeMinThrottle,
    RcAttitudeMode,
    RcMaxRoll,
    RcMaxPitch,
    RcMaxRollRate,
    RcMaxPitchRate,
    RcMaxYawRate,
    Mixer,
    FixedWing,
    ElevatorReverse,
    AileronReverse,
    RudderReverse,
    FcRoll,
    FcPitch,
    FcYaw,
    ArmThreshold,
    OffboardTimeout,
    BatteryVoltageMultiplier,
    BatteryCurrentMultiplier,
    BatteryVoltageAlpha,
    BatteryCurrentAlpha
);

impl Setter for BaudRate {
    fn set<'a>(&mut self, input: <Self as Callback>::Args<'a>) {
        let (callee, data) = input;
        self.value = data;
        callee.do_something(data);
    }
}

pub struct TestStruct {
    pub val: i32,
    pub val2: i32,
}

impl TestStruct {
    fn do_something(&mut self, input: ParamValue) {
        match input {
            ParamValue::Int(val) => self.val = val,
            _ => (),
        }
    }
}