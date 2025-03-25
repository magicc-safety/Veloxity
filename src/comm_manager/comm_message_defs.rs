use super::super::params::ParamValue;
pub const PARAM_NAME_LENGTH: usize = 25;

#[derive(Copy, Clone)]
pub enum RosflightAuxCmdType {
    Disabled,
    Servo,
    Motor,
    End,
}

#[derive(Copy, Clone)]
pub enum RosflightCmdResponse {
    Failed,
    Success,
    End,
}

#[derive(Clone, Copy)]
pub enum OffBoardControlMode {
    PassThrough,
    RollratePitchrateYawrateThrottle,
    RollPitchYawrateThrotte,
    RollPitchYawrateAltitude,
    XvelYvelYawrateAltitude,
    XposYposYawAltitude,
    End,
}

#[derive(Clone, Copy)]
pub enum OffboardControlIgnore {
    IgnoreNone = 0x0,
    IgnoreVal1 = 0x1,
    IgnoreVal2 = 0x2,
    IgnoreVal3 = 0x4,
    IgnoreVal4 = 0x8,
    IgnoreVal5 = 0x10,
    IgnoreVal6 = 0x20,
    End,
}

#[derive(Copy, Clone)]
pub enum RosflightRangeType {
    Sonar,
    Lidar,
    End
}

#[derive(Clone, Copy)]
pub struct Channel {
    pub value: f32,
    pub valid: bool,
}

#[derive(Copy, Clone)]
pub struct OffBoardControl {
    pub U: [Channel; 6],
}

#[derive(Copy, Clone)]
pub struct ParameterRequestRead { // id is a field used in rosflight... structure of params is different so we won't care to use id
    pub name_len: usize,
    pub name: [u8; PARAM_NAME_LENGTH]
}

#[derive(Copy, Clone)]
pub struct ParamSet {
    pub value: ParamValue,
    pub name_len: usize,
    pub name: [u8; PARAM_NAME_LENGTH]
}

#[derive(Copy, Clone)]
pub struct ExternalAttitude {
    pub q: [f32; 4]
}

#[derive(Copy, Clone)]
pub enum CommMessageCommand {
    RcCalibration,
    AccelCalibration,
    GyroCalibration,
    BaroCalibration,
    AirspeedCalibration,
    ReadParams,
    WriteParams,
    SetParamDefaults,
    Reboot,
    RebootToBootloader,
    SendVersion,
    ResetOrigin,
    SendAllConfigInfos,
    End,
}

#[derive(Copy, Clone)]
pub struct RosflightCmd {
    pub command: CommMessageCommand,
    pub success: bool,
}

#[derive(Copy, Clone)]
pub struct TEMP_mixer_aux_command {
    pub foo: bool,
}

#[derive(Copy, Clone)]
pub struct Timesync {
    pub local: u64,
    pub remote: u64,
}

#[derive(Clone, Copy)]
pub enum CommMessage {
    OffBoardControl(OffBoardControl),
    // ParamRequestList,
    ParamRequestRead(ParameterRequestRead),
    ParamSet(ParamSet),
    RosflightCmd(RosflightCmd),
    RosflightAuxCmd(TEMP_mixer_aux_command),
    Timesync(Timesync),
    ExternalAttitude(ExternalAttitude),
    Heartbeat,
    End,
}
