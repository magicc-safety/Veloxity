use bitflags::bitflags;
use heapless::Deque;
use messages::*;

// PARAM_SET has a two-stage path: decoded MAVLink ingress waits here, then the
// comm system admits work into the ECS event queue while that queue has room.
// Keep enough room for a companion computer to send a full parameter-table load
// as one burst without dropping valid set requests.
pub const PARAM_SET_BURST_QUEUE_CAPACITY: usize = 512;
pub const PARAM_SET_INGRESS_QUEUE_CAPACITY: usize = PARAM_SET_BURST_QUEUE_CAPACITY;
pub const PARAM_SET_EVENT_QUEUE_CAPACITY: usize = PARAM_SET_BURST_QUEUE_CAPACITY;
pub const PARAM_READ_INGRESS_QUEUE_CAPACITY: usize = PARAM_SET_BURST_QUEUE_CAPACITY;

#[derive(Default)]
pub struct Messages {
    pub heartbeat: Option<HeartbeatMsg>,
    pub param_request_read: Deque<ParamRequestReadMsg, PARAM_READ_INGRESS_QUEUE_CAPACITY>,
    pub param_request_list: Option<ParamRequestListMsg>,
    pub param_set: Deque<ParamSetMsg, PARAM_SET_INGRESS_QUEUE_CAPACITY>,
    pub timesync: Option<TimesyncMsg>,
    pub offboard_control: Option<OffboardControlMsg>,
    pub cmd: Option<RosflightCmdMsg>,
    pub aux_cmd: Option<RosflightAuxCmdMsg>,
    pub external_attitude: Option<ExternalAttitudeMsg>,
    pub rc_raw: Option<RcChannelsMsg>,
}

impl Messages {
    pub fn has_pending(&self) -> bool {
        self.heartbeat.is_some()
            || !self.param_request_read.is_empty()
            || self.param_request_list.is_some()
            || !self.param_set.is_empty()
            || self.timesync.is_some()
            || self.offboard_control.is_some()
            || self.cmd.is_some()
            || self.aux_cmd.is_some()
            || self.external_attitude.is_some()
            || self.rc_raw.is_some()
    }
}

pub trait Store<T> {
    fn store(&mut self, msg: T);
    fn take(&mut self) -> Option<T>;
}

// implements store function for each message type. comm should only receive known messages
macro_rules! impl_store {
    ($ty:ty, $field:ident, $name:literal) => {
        impl Store<$ty> for Messages {
            fn store(&mut self, msg: $ty) {
                let _ = self.$field.insert(msg);
            }
            fn take(&mut self) -> Option<$ty> {
                self.$field.take()
            }
        }
    };
}

// implemented for messages that will be received
impl_store!(HeartbeatMsg, heartbeat, "heartbeat");
impl Store<ParamRequestReadMsg> for Messages {
    fn store(&mut self, msg: ParamRequestReadMsg) {
        if self.param_request_read.push_back(msg).is_err() {
            crate::log_warn!("message queue full: param_request_read");
        }
    }

    fn take(&mut self) -> Option<ParamRequestReadMsg> {
        self.param_request_read.pop_front()
    }
}
impl_store!(
    ParamRequestListMsg,
    param_request_list,
    "param_request_list"
);
impl Store<ParamSetMsg> for Messages {
    fn store(&mut self, msg: ParamSetMsg) {
        if self.param_set.push_back(msg).is_err() {
            crate::log_warn!("message queue full: param_set");
        }
    }

    fn take(&mut self) -> Option<ParamSetMsg> {
        self.param_set.pop_front()
    }
}
impl_store!(TimesyncMsg, timesync, "timesync");
impl_store!(OffboardControlMsg, offboard_control, "offboard_control");
impl_store!(RosflightCmdMsg, cmd, "cmd");
impl_store!(RosflightAuxCmdMsg, aux_cmd, "aux_cmd");
impl_store!(ExternalAttitudeMsg, external_attitude, "external_attitude");

pub mod messages {
    use super::enums::*;
    use crate::{packets::GNSSFixType, params::ParamValue, state_machine::ErrorFlag};
    // Heartbeat
    // I don't think we need all these fields for the generic message but I'm leaving them for now
    #[derive(Debug, Clone, Copy)]
    pub struct HeartbeatMsg {
        pub type_: u8,     // MAV_TYPE
        pub autopilot: u8, // MAV_AUTOPILOT (not found in xml...)
        pub base_mode: u8, // MAV_MODE_FLAG
        pub custom_mode: u32,
        pub system_status: u8,   // MAV_STATE
        pub mavlink_version: u8, // V1
    }

    // Note I changed the MAVLink param messages to use ParamValue. These may need to change to fit the param system

    #[derive(Debug, Clone, Copy)]
    pub struct ParamRequestReadMsg {
        pub target_system: u8,
        pub target_component: u8,
        pub param_identifier: ParamIdentifier,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct ParamRequestListMsg {
        pub target_system: u8,
        pub target_component: u8,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct ParamValueMsg {
        pub param_id: [u8; 16],
        pub param_value: ParamValue,
        pub param_count: u16,
        pub param_index: u16,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct ParamSetMsg {
        pub target_system: u8,
        pub target_component: u8,
        pub param_id: [u8; 16],
        pub param_value: ParamValue,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct AttitudeQuaternionMsg {
        pub time_boot_ms: u32,
        pub q1: f32,         // w
        pub q2: f32,         // x
        pub q3: f32,         // y
        pub q4: f32,         // z
        pub rollspeed: f32,  // (rad/s)
        pub pitchspeed: f32, // (rad/s)
        pub yawspeed: f32,   // (rad/s)
    }

    // This could be handled differently... choosing this for now. Used RC packet for ref
    pub const RC_PACKET_CHANNELS: usize = 24;
    #[derive(Debug, Clone, Copy)]
    pub struct RcChannelsMsg {
        pub time_boot_ms: u32,
        pub chancount: u8,
        pub channels: [u16; RC_PACKET_CHANNELS],
        pub rssi: u8,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct TimesyncMsg {
        pub tc1: i64,
        pub ts1: i64,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct StatustextMsg {
        pub severity: Severity,
        pub text: [u8; 50],
    }

    // Custom ROSflight messages below here. Should be good to go out of the box

    #[derive(Debug, Clone, Copy)]
    pub struct OffboardControlMsg {
        pub mode: OffboardControlMode,
        pub ignore: OffboardControlIgnore,
        pub qx: f32,
        pub qy: f32,
        pub qz: f32,
        pub fx: f32,
        pub fy: f32,
        pub fz: f32,
        pub passthrough: [f32; 4],
    }

    #[derive(Debug, Clone, Copy)]
    pub struct SmallImuMsg {
        pub time_boot_us: u64,
        pub xacc: f32,
        pub yacc: f32,
        pub zacc: f32,
        pub xgyro: f32,
        pub ygyro: f32,
        pub zgyro: f32,
        pub temperature: f32,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct SmallMagMsg {
        pub xmag: f32,
        pub ymag: f32,
        pub zmag: f32,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct SmallBaroMsg {
        pub altitude: f32,    // (m)
        pub pressure: f32,    // (Pa)
        pub temperature: f32, // (K)
    }

    #[derive(Debug, Clone, Copy)]
    pub struct DiffPressureMsg {
        pub velocity: f32,      // (m/s)
        pub diff_pressure: f32, // (Pa)
        pub temperature: f32,   // (K)
    }

    #[derive(Debug, Clone, Copy)]
    pub struct SmallRangeMsg {
        pub type_: RosflightRangeType,
        pub range: f32,     // (m)
        pub max_range: f32, // (m)
        pub min_range: f32, // (m)
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RosflightCmdMsg {
        pub command: RosflightCmd,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RosflightCmdAckMsg {
        pub command: RosflightCmd,
        pub success: RosflightCmdResponse,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RosflightOutputRawMsg {
        pub stamp: u64,
        pub values: [f32; 14],
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RosflightStatusMsg {
        pub armed: u8,
        pub failsafe: u8,
        pub rc_override: u16,
        pub offboard: u8,
        pub error_code: ErrorFlag,
        pub control_mode: OffboardControlMode,
        pub num_errors: i16,
        pub loop_time_us: i16,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RosflightVersionMsg {
        pub version: [u8; 50],
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RosflightAuxCmdMsg {
        pub type_array: [RosflightAuxCmdType; 14usize],
        pub aux_cmd_array: [f32; 14],
    }

    #[derive(Debug, Clone, Copy)]
    pub struct ExternalAttitudeMsg {
        pub qw: f32,
        pub qx: f32,
        pub qy: f32,
        pub qz: f32,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RosflightHardErrorMsg {
        pub error_code: u32,
        pub pc: u32,
        pub reset_count: u32,
        pub do_rearm: u32,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RosflightGnssMsg {
        pub seconds: i64,
        pub nanos: i32,
        pub fix_type: GNSSFixType,
        pub num_sat: u8,
        pub lat: f64,                 // deg DDS format
        pub lon: f64,                 // deg DDS format
        pub height: f32,              // (m)
        pub vel_n: f32,               // (m/s)
        pub vel_e: f32,               // (m/s)
        pub vel_d: f32,               // (m/s)
        pub h_acc: f32,               // (m)
        pub v_acc: f32,               // (m)
        pub s_acc: f32,               // (m)
        pub rosflight_timestamp: u64, // us, estimated firmware timestamp for the time of validity of the gnss
    }

    #[derive(Debug, Clone, Copy)]
    pub struct BatteryStatusMsg {
        pub battery_voltage: f32,
        pub battery_current: f32,
    }

    #[derive(Debug, Clone, Copy)]
    pub enum DownlinkMessage {
        Heartbeat(HeartbeatMsg),
        ParamValue(ParamValueMsg),
        Status(RosflightStatusMsg),
        Timesync(TimesyncMsg),
        Version(RosflightVersionMsg),
        OutputRaw(RosflightOutputRawMsg),
        Attitude(AttitudeQuaternionMsg),
        Baro(SmallBaroMsg),
        DiffPressure(DiffPressureMsg),
        Imu(SmallImuMsg),
        Mag(SmallMagMsg),
        RcRaw(RcChannelsMsg),
        Range(SmallRangeMsg),
        Gnss(RosflightGnssMsg),
        CmdAck(RosflightCmdAckMsg),
        RcChannels(RcChannelsMsg),
        BatteryStatus(BatteryStatusMsg),
        Statustext(StatustextMsg),
        HardError(RosflightHardErrorMsg),
    }
}

// Enums

pub mod enums {
    use super::bitflags;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum RosflightCmd {
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
    }

    #[derive(Debug, Clone, Copy)]
    pub enum RosflightCmdResponse {
        RosflightCmdFailed,
        RosflightCmdSuccess,
    }

    #[repr(u8)]
    #[derive(Clone, Copy, Debug, PartialEq, Default)]
    pub enum OffboardControlMode {
        ModePassThrough = 0,
        ModeRollratePitchrateYawrateThrottle = 1,
        #[default]
        ModeRollPitchYawrateThrottle = 2,
    }

    #[derive(Debug, Clone, Copy)]
    pub enum RosflightAuxCmdType {
        Disabled,
        Servo,
        Motor,
    }

    #[derive(Debug, Clone, Copy)]
    pub enum Severity {
        Emergency,
        Alert,
        Critical,
        Error,
        Warning,
        Notice,
        Info,
        Debug,
    }

    bitflags! {
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
        pub struct OffboardControlIgnore: u16 {
            const IGNORE_FX = 1 << 0;
            const IGNORE_FY = 1 << 1;
            const IGNORE_FZ = 1 << 2;
            const IGNORE_QX = 1 << 3;
            const IGNORE_QY = 1 << 4;
            const IGNORE_QZ = 1 << 5;
            const IGNORE_PASS_0 = 1 << 6;
            const IGNORE_PASS_1 = 1 << 7;
            const IGNORE_PASS_2 = 1 << 8;
            const IGNORE_PASS_3 = 1 << 9;
        }
    }

    impl OffboardControlIgnore {
        pub fn is_ignoring_qx(&self) -> bool {
            self.intersects(Self::IGNORE_QX)
        }

        pub fn is_ignoring_qy(&self) -> bool {
            self.intersects(Self::IGNORE_QY)
        }

        pub fn is_ignoring_qz(&self) -> bool {
            self.intersects(Self::IGNORE_QZ)
        }

        pub fn is_ignoring_fx(&self) -> bool {
            self.intersects(Self::IGNORE_FX)
        }

        pub fn is_ignoring_fy(&self) -> bool {
            self.intersects(Self::IGNORE_FY)
        }

        pub fn is_ignoring_fz(&self) -> bool {
            self.intersects(Self::IGNORE_FZ)
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub enum GnssFixType {
        GnssFixNoFix,
        GnssFixDeadReckoningOnly,
        GnssFix2dFix,
        GnssFix3dFix,
        GnssFixGnssPlusDeadReckoning,
        GnssFixTimeFixOnly,
    }

    #[derive(Debug, Clone, Copy, Default)]
    pub enum RosflightRangeType {
        #[default]
        RosflightRangeSonar,
        RosflightRangeLidar,
    }

    #[derive(Debug, Clone, Copy)]
    pub enum MavType {
        Generic,
        FixedWing,
        Quadrotor,
    }

    #[derive(Debug, Clone, Copy, Default)]
    pub enum LogLevel {
        #[default]
        Info,
        Warn,
        Error,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum ParamIdentifier {
        ID([u8; 16]),
        INDEX(i16),
    }
}
