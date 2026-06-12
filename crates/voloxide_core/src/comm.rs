pub mod interface;
pub mod messages;

use crate::board;
use crate::comm::messages::{Messages, Store, enums::*, messages::*};
use crate::command::CommandManager;
use crate::estimator::AttitudeEstimate;
use crate::events::{
    AuxCommandReceived, BoardCommandRequested, CalibrationRequested, CommEventQueues, CommResponse,
    CommandEventQueues, CompanionEventQueues, CompanionHeartbeatReceived, ConfigInfoRequested,
    ExternalAttitudeReceived, OffboardControlRequested, ParamDefaultsRequested, ParamEventQueues,
    ParamListRequested, ParamReadRequested, ParamSetRequested, RcTrimCalibrationRequested,
    ResetOriginRequested, VersionRequested,
};
use crate::math::FlightFloat;
use crate::packets::{RC_PACKET_CHANNELS, RangeType};
use crate::params::{ParamId, ParamValue, Params};
use crate::sensors::ProcessedSensors;
use crate::state_machine::StateManager;
#[cfg(feature = "timing-diagnostics")]
use core::fmt::Write;
use core::marker::PhantomData;
#[cfg(feature = "timing-diagnostics")]
use core::sync::atomic::{AtomicU32, Ordering};
#[cfg(feature = "timing-diagnostics")]
use heapless::String;

const MAV_TYPE_FIXED_WING: u8 = 1;
const MAV_TYPE_QUADROTOR: u8 = 2;
const OUTPUT_RAW_IMU_DIVISOR: u64 = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NamedTelemetryStream {
    Heartbeat,
    Status,
    Imu,
    Rc,
    Attitude,
    OutputRaw,
    Gnss,
    DiffPressure,
    Baro,
    Mag,
    Range,
    Battery,
}

#[cfg(feature = "timing-diagnostics")]
const NAMED_TELEMETRY_STREAM_COUNT: usize = 12;

#[cfg(feature = "timing-diagnostics")]
impl NamedTelemetryStream {
    const fn index(self) -> usize {
        match self {
            Self::Heartbeat => 0,
            Self::Status => 1,
            Self::Imu => 2,
            Self::Rc => 3,
            Self::Attitude => 4,
            Self::OutputRaw => 5,
            Self::Gnss => 6,
            Self::DiffPressure => 7,
            Self::Baro => 8,
            Self::Mag => 9,
            Self::Range => 10,
            Self::Battery => 11,
        }
    }

    const fn label(self) -> &'static str {
        match self {
            Self::Heartbeat => "H",
            Self::Status => "S",
            Self::Imu => "I",
            Self::Rc => "R",
            Self::Attitude => "A",
            Self::OutputRaw => "O",
            Self::Gnss => "G",
            Self::DiffPressure => "D",
            Self::Baro => "B",
            Self::Mag => "M",
            Self::Range => "N",
            Self::Battery => "P",
        }
    }

    const fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(Self::Heartbeat),
            1 => Some(Self::Status),
            2 => Some(Self::Imu),
            3 => Some(Self::Rc),
            4 => Some(Self::Attitude),
            5 => Some(Self::OutputRaw),
            6 => Some(Self::Gnss),
            7 => Some(Self::DiffPressure),
            8 => Some(Self::Baro),
            9 => Some(Self::Mag),
            10 => Some(Self::Range),
            11 => Some(Self::Battery),
            _ => None,
        }
    }
}

#[cfg(feature = "timing-diagnostics")]
static TELEMETRY_STREAM_DUE: [AtomicU32; NAMED_TELEMETRY_STREAM_COUNT] =
    [const { AtomicU32::new(0) }; NAMED_TELEMETRY_STREAM_COUNT];
#[cfg(feature = "timing-diagnostics")]
static TELEMETRY_STREAM_SELECTED: [AtomicU32; NAMED_TELEMETRY_STREAM_COUNT] =
    [const { AtomicU32::new(0) }; NAMED_TELEMETRY_STREAM_COUNT];
#[cfg(feature = "timing-diagnostics")]
static TELEMETRY_STREAM_SENT: [AtomicU32; NAMED_TELEMETRY_STREAM_COUNT] =
    [const { AtomicU32::new(0) }; NAMED_TELEMETRY_STREAM_COUNT];
#[cfg(feature = "timing-diagnostics")]
static TELEMETRY_STREAM_FAILED: [AtomicU32; NAMED_TELEMETRY_STREAM_COUNT] =
    [const { AtomicU32::new(0) }; NAMED_TELEMETRY_STREAM_COUNT];

#[cfg(feature = "timing-diagnostics")]
fn record_telemetry_stream_due(stream: NamedTelemetryStream) {
    TELEMETRY_STREAM_DUE[stream.index()].fetch_add(1, Ordering::Relaxed);
}

#[cfg(feature = "timing-diagnostics")]
fn record_telemetry_stream_selected(stream: NamedTelemetryStream) {
    TELEMETRY_STREAM_SELECTED[stream.index()].fetch_add(1, Ordering::Relaxed);
}

#[cfg(feature = "timing-diagnostics")]
fn record_telemetry_stream_sent(stream: NamedTelemetryStream) {
    TELEMETRY_STREAM_SENT[stream.index()].fetch_add(1, Ordering::Relaxed);
}

#[cfg(feature = "timing-diagnostics")]
fn record_telemetry_stream_failed(stream: NamedTelemetryStream) {
    TELEMETRY_STREAM_FAILED[stream.index()].fetch_add(1, Ordering::Relaxed);
}

#[cfg(feature = "timing-diagnostics")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuTelemetryReadiness {
    Due,
    NotDue,
    Stale,
    NoImu,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TelemetryRates {
    pub heartbeat_hz: u16,
    pub status_hz: u16,
    pub imu_hz: u16,
    pub attitude_hz: u16,
    pub output_raw_hz: u16,
    pub diff_pressure_hz: u16,
    pub baro_hz: u16,
    pub mag_hz: u16,
    pub range_hz: u16,
    pub battery_hz: u16,
    pub gnss_hz: u16,
    pub rc_hz: u16,
    pub output_raw_imu_divisor: u64,
}

impl TelemetryRates {
    pub const fn upstream() -> Self {
        Self {
            heartbeat_hz: 1,
            status_hz: 10,
            imu_hz: 0,
            attitude_hz: 0,
            output_raw_hz: 0,
            diff_pressure_hz: 0,
            baro_hz: 0,
            mag_hz: 0,
            range_hz: 0,
            battery_hz: 0,
            gnss_hz: 0,
            rc_hz: 0,
            output_raw_imu_divisor: OUTPUT_RAW_IMU_DIVISOR,
        }
    }

    pub const fn bounded_high_rate_transport() -> Self {
        Self {
            heartbeat_hz: 1,
            status_hz: 10,
            imu_hz: 400,
            attitude_hz: 50,
            output_raw_hz: 50,
            diff_pressure_hz: 50,
            baro_hz: 25,
            mag_hz: 25,
            range_hz: 50,
            battery_hz: 25,
            gnss_hz: 10,
            rc_hz: 100,
            output_raw_imu_divisor: 0,
        }
    }
}

impl Default for TelemetryRates {
    fn default() -> Self {
        Self::upstream()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct TelemetryRateState {
    imu_us: u64,
    attitude_us: u64,
    output_raw_us: u64,
    diff_pressure_us: u64,
    baro_us: u64,
    mag_us: u64,
    range_us: u64,
    battery_us: u64,
    gnss_us: u64,
    rc_us: u64,
}

fn stream_due(now_us: u64, last_us: &mut u64, rate_hz: u16) -> bool {
    if rate_hz == 0 {
        if *last_us == 0 {
            *last_us = now_us;
        }
        return true;
    }

    let interval_us = 1_000_000_u64 / rate_hz as u64;
    if *last_us == 0 {
        *last_us = now_us;
        true
    } else if now_us.saturating_sub(*last_us) >= interval_us {
        let elapsed_intervals = now_us.saturating_sub(*last_us) / interval_us;
        *last_us = last_us.saturating_add(elapsed_intervals.saturating_mul(interval_us));
        true
    } else {
        false
    }
}

fn stream_due_deadline_us(now_us: u64, last_us: u64, rate_hz: u16) -> Option<u64> {
    if rate_hz == 0 {
        return Some(if last_us == 0 { 0 } else { now_us });
    }
    if last_us == 0 {
        return Some(0);
    }

    let interval_us = 1_000_000_u64 / rate_hz as u64;
    let deadline_us = last_us.saturating_add(interval_us);
    let elapsed_us = now_us.saturating_sub(last_us);
    if elapsed_us >= interval_us {
        Some(deadline_us)
    } else {
        None
    }
}

fn fixed_rate_due(now_us: u64, last_us: u64, rate_hz: u16) -> bool {
    if rate_hz == 0 {
        return false;
    }

    let interval_us = 1_000_000_u64 / rate_hz as u64;
    now_us.saturating_sub(last_us) >= interval_us
}

fn fixed_rate_due_deadline_us(now_us: u64, last_us: u64, rate_hz: u16) -> Option<u64> {
    if rate_hz == 0 {
        return None;
    }

    let interval_us = 1_000_000_u64 / rate_hz as u64;
    let deadline_us = last_us.saturating_add(interval_us);
    let elapsed_us = now_us.saturating_sub(last_us);
    if elapsed_us >= interval_us {
        Some(deadline_us)
    } else {
        None
    }
}

pub const fn str_to_fixed_bytes(input: &str) -> [u8; 16] {
    let mut buffer = [0u8; 16];
    let input_bytes = input.as_bytes();

    // Determine how many bytes to copy (at most 16)
    let len_to_copy = if input_bytes.len() > 16 {
        16
    } else {
        input_bytes.len()
    };

    // Copy the bytes from the input string
    let mut i = 0;
    while i < len_to_copy {
        buffer[i] = input_bytes[i];
        i += 1;
    }

    // If the input was shorter than 16, the spot after the last character
    // is already a 0 from the initial buffer creation, so it is null-terminated.
    // If the input was 16 or longer, the buffer is full and not null-terminated.

    buffer
}

fn param_int(params: &Params, id: ParamId) -> i32 {
    match params.get_by_id(id) {
        ParamValue::Int(value) => value,
        _ => 0,
    }
}

pub struct CommManager<B, T>
where
    B: board::BoardIo,
    T: interface::CommInterface<B>,
{
    last_heartbeat_us: u64,
    last_status_send_us: u64,
    output_raw_imu_count: u64,
    telemetry_rates: TelemetryRates,
    telemetry_rate_state: TelemetryRateState,
    last_realtime_imu_telemetry_timestamp: Option<u64>,
    #[cfg(feature = "timing-diagnostics")]
    telemetry_diag_text_index: usize,

    pub sysid: u8,
    comm_link: T,
    pub msgs: Messages,
    _board_marker: PhantomData<B>,
}

impl<B, T> CommManager<B, T>
where
    B: board::BoardIo,
    T: interface::CommInterface<B>,
{
    pub fn new(comm_link: T, now_us: u64) -> Self {
        CommManager {
            last_heartbeat_us: now_us,
            last_status_send_us: now_us,
            output_raw_imu_count: 0,
            telemetry_rates: TelemetryRates::upstream(),
            telemetry_rate_state: TelemetryRateState::default(),
            last_realtime_imu_telemetry_timestamp: None,
            #[cfg(feature = "timing-diagnostics")]
            telemetry_diag_text_index: 0,

            sysid: 1,
            comm_link,
            msgs: Messages::default(),
            _board_marker: PhantomData,
        }
    }

    #[cfg(test)]
    pub(crate) fn comm_link(&self) -> &T {
        &self.comm_link
    }

    pub fn process_incoming_messages(&mut self, board: &mut B) {
        self.comm_link
            .handle_incoming_messages(board, &mut self.msgs);
    }

    pub fn has_pending_messages(&self) -> bool {
        self.msgs.has_pending()
    }

    pub fn set_telemetry_rates(&mut self, telemetry_rates: TelemetryRates) {
        self.telemetry_rates = telemetry_rates;
        self.telemetry_rate_state = TelemetryRateState::default();
        self.output_raw_imu_count = 0;
        self.last_realtime_imu_telemetry_timestamp = None;
    }

    pub fn named_telemetry_due<R>(
        &self,
        now_us: u64,
        processed_sensors: &ProcessedSensors<R>,
    ) -> bool
    where
        R: FlightFloat,
    {
        self.select_due_named_telemetry_stream(now_us, processed_sensors, false)
            .is_some()
    }

    #[cfg(feature = "timing-diagnostics")]
    pub fn imu_telemetry_readiness<R>(
        &self,
        now_us: u64,
        processed_sensors: &ProcessedSensors<R>,
    ) -> ImuTelemetryReadiness
    where
        R: FlightFloat,
    {
        let Some(imu_packet) = processed_sensors.imu else {
            return ImuTelemetryReadiness::NoImu;
        };
        if self.last_realtime_imu_telemetry_timestamp == Some(imu_packet.header.timestamp) {
            return ImuTelemetryReadiness::Stale;
        }
        if stream_due_deadline_us(
            now_us,
            self.telemetry_rate_state.imu_us,
            self.telemetry_rates.imu_hz,
        )
        .is_some()
        {
            ImuTelemetryReadiness::Due
        } else {
            ImuTelemetryReadiness::NotDue
        }
    }

    fn select_due_named_telemetry_stream<R>(
        &self,
        now_us: u64,
        processed_sensors: &ProcessedSensors<R>,
        record_diagnostics: bool,
    ) -> Option<NamedTelemetryStream>
    where
        R: FlightFloat,
    {
        #[cfg(not(feature = "timing-diagnostics"))]
        let _ = record_diagnostics;
        let mut selected = None;
        let mut selected_deadline = u64::MAX;

        let consider = |selected: &mut Option<NamedTelemetryStream>,
                        selected_deadline: &mut u64,
                        stream: NamedTelemetryStream| {
            let Some(deadline) =
                self.named_telemetry_stream_deadline(stream, now_us, processed_sensors)
            else {
                return;
            };
            #[cfg(feature = "timing-diagnostics")]
            if record_diagnostics {
                record_telemetry_stream_due(stream);
            }
            if selected.is_none() || deadline < *selected_deadline {
                *selected = Some(stream);
                *selected_deadline = deadline;
            }
        };

        consider(
            &mut selected,
            &mut selected_deadline,
            NamedTelemetryStream::Heartbeat,
        );
        consider(
            &mut selected,
            &mut selected_deadline,
            NamedTelemetryStream::Status,
        );

        if processed_sensors.imu.is_some() {
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::Imu,
            );
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::Attitude,
            );
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::OutputRaw,
            );
        }

        if processed_sensors.rc.is_some() {
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::Rc,
            );
        }
        if processed_sensors.gnss.is_some() {
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::Gnss,
            );
        }
        if processed_sensors.pitot.is_some() {
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::DiffPressure,
            );
        }
        if processed_sensors.baro.is_some() {
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::Baro,
            );
        }
        if processed_sensors.mag.is_some() {
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::Mag,
            );
        }
        if processed_sensors.range.is_some() {
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::Range,
            );
        }
        if processed_sensors.battery.is_some() {
            consider(
                &mut selected,
                &mut selected_deadline,
                NamedTelemetryStream::Battery,
            );
        }

        selected
    }

    fn named_telemetry_stream_deadline<R>(
        &self,
        stream: NamedTelemetryStream,
        now_us: u64,
        processed_sensors: &ProcessedSensors<R>,
    ) -> Option<u64>
    where
        R: FlightFloat,
    {
        match stream {
            NamedTelemetryStream::Heartbeat => fixed_rate_due_deadline_us(
                now_us,
                self.last_heartbeat_us,
                self.telemetry_rates.heartbeat_hz,
            ),
            NamedTelemetryStream::Status => fixed_rate_due_deadline_us(
                now_us,
                self.last_status_send_us,
                self.telemetry_rates.status_hz,
            ),
            NamedTelemetryStream::Imu => {
                let imu_packet = processed_sensors.imu?;
                if self.last_realtime_imu_telemetry_timestamp == Some(imu_packet.header.timestamp) {
                    None
                } else {
                    stream_due_deadline_us(
                        now_us,
                        self.telemetry_rate_state.imu_us,
                        self.telemetry_rates.imu_hz,
                    )
                }
            }
            NamedTelemetryStream::Rc => processed_sensors.rc.as_ref().and_then(|_| {
                stream_due_deadline_us(
                    now_us,
                    self.telemetry_rate_state.rc_us,
                    self.telemetry_rates.rc_hz,
                )
            }),
            NamedTelemetryStream::Attitude => processed_sensors.imu.as_ref().and_then(|_| {
                stream_due_deadline_us(
                    now_us,
                    self.telemetry_rate_state.attitude_us,
                    self.telemetry_rates.attitude_hz,
                )
            }),
            NamedTelemetryStream::OutputRaw => processed_sensors.imu.as_ref().and_then(|_| {
                if self.telemetry_rates.output_raw_hz == 0 {
                    (self.telemetry_rates.output_raw_imu_divisor != 0
                        && self.output_raw_imu_count % self.telemetry_rates.output_raw_imu_divisor
                            == 0)
                        .then_some(now_us)
                } else {
                    stream_due_deadline_us(
                        now_us,
                        self.telemetry_rate_state.output_raw_us,
                        self.telemetry_rates.output_raw_hz,
                    )
                }
            }),
            NamedTelemetryStream::Gnss => processed_sensors.gnss.as_ref().and_then(|_| {
                stream_due_deadline_us(
                    now_us,
                    self.telemetry_rate_state.gnss_us,
                    self.telemetry_rates.gnss_hz,
                )
            }),
            NamedTelemetryStream::DiffPressure => processed_sensors.pitot.as_ref().and_then(|_| {
                stream_due_deadline_us(
                    now_us,
                    self.telemetry_rate_state.diff_pressure_us,
                    self.telemetry_rates.diff_pressure_hz,
                )
            }),
            NamedTelemetryStream::Baro => processed_sensors.baro.as_ref().and_then(|_| {
                stream_due_deadline_us(
                    now_us,
                    self.telemetry_rate_state.baro_us,
                    self.telemetry_rates.baro_hz,
                )
            }),
            NamedTelemetryStream::Mag => processed_sensors.mag.as_ref().and_then(|_| {
                stream_due_deadline_us(
                    now_us,
                    self.telemetry_rate_state.mag_us,
                    self.telemetry_rates.mag_hz,
                )
            }),
            NamedTelemetryStream::Range => processed_sensors.range.as_ref().and_then(|_| {
                stream_due_deadline_us(
                    now_us,
                    self.telemetry_rate_state.range_us,
                    self.telemetry_rates.range_hz,
                )
            }),
            NamedTelemetryStream::Battery => processed_sensors.battery.as_ref().and_then(|_| {
                stream_due_deadline_us(
                    now_us,
                    self.telemetry_rate_state.battery_us,
                    self.telemetry_rates.battery_hz,
                )
            }),
        }
    }

    fn targets_this_system(&self, target_system: u8) -> bool {
        target_system == self.sysid
    }

    pub fn send_named_telemetry_streams<S, A, R>(
        &mut self,
        board: &mut B,
        now_us: u64,
        state_manager: &StateManager,
        command_manager: &CommandManager,
        params: &Params,
        estimator_state: &S,
        processed_sensors: &ProcessedSensors<R>,
        actuator_commands: &A,
        sensor_error_count: u16,
        loop_time_us: u16,
    ) where
        S: AttitudeEstimate,
        A: AsRef<[R]>,
        R: FlightFloat,
    {
        if fixed_rate_due(
            now_us,
            self.last_heartbeat_us,
            self.telemetry_rates.heartbeat_hz,
        ) {
            self.send_rosflight_heartbeat(
                board,
                HeartbeatMsg {
                    autopilot: 0,
                    base_mode: 0,
                    custom_mode: 0,
                    mavlink_version: 0,
                    system_status: 0,
                    type_: if param_int(params, ParamId::PARAM_FIXED_WING) != 0 {
                        MAV_TYPE_FIXED_WING
                    } else {
                        MAV_TYPE_QUADROTOR
                    },
                },
            );
            self.last_heartbeat_us = now_us;
        }

        if fixed_rate_due(
            now_us,
            self.last_status_send_us,
            self.telemetry_rates.status_hz,
        ) {
            self.send_rosflight_status(
                board,
                RosflightStatusMsg {
                    armed: state_manager.is_armed() as u8,
                    failsafe: state_manager.is_in_failsafe() as u8,
                    rc_override: command_manager.get_rc_override(),
                    offboard: command_manager.is_offboard_active() as u8,
                    error_code: state_manager.get_errors(),
                    control_mode: command_manager.get_control_mode().into(),
                    num_errors: sensor_error_count as i16,
                    loop_time_us: loop_time_us as i16,
                },
            );
            self.last_status_send_us = now_us;
        }

        if let Some(imu_packet) = processed_sensors.imu {
            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.imu_us,
                self.telemetry_rates.imu_hz,
            ) {
                self.send_rosflight_small_imu(
                    board,
                    SmallImuMsg {
                        temperature: imu_packet.temperature,
                        time_boot_us: imu_packet.header.timestamp,
                        xacc: imu_packet.accel[0].to_f32_lossy(),
                        yacc: imu_packet.accel[1].to_f32_lossy(),
                        zacc: imu_packet.accel[2].to_f32_lossy(),
                        xgyro: imu_packet.gyro[0].to_f32_lossy(),
                        ygyro: imu_packet.gyro[1].to_f32_lossy(),
                        zgyro: imu_packet.gyro[2].to_f32_lossy(),
                    },
                );
            }

            let q = estimator_state.q();
            let qd = estimator_state.q_dot();
            let rollspeed = 2.0 * (q[0] * qd[1] - q[1] * qd[0] - q[2] * qd[3] + q[3] * qd[2]);
            let pitchspeed = 2.0 * (q[0] * qd[2] - q[1] * qd[3] - q[2] * qd[0] + q[3] * qd[1]);
            let yawspeed = 2.0 * (q[0] * qd[3] - q[1] * qd[2] - q[2] * qd[1] + q[3] * qd[0]);

            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.attitude_us,
                self.telemetry_rates.attitude_hz,
            ) {
                self.send_rosflight_attitude_quaternion(
                    board,
                    AttitudeQuaternionMsg {
                        time_boot_ms: (imu_packet.header.timestamp / 1000) as u32,
                        q1: q[0],
                        q2: q[1],
                        q3: q[2],
                        q4: q[3],
                        rollspeed,
                        pitchspeed,
                        yawspeed,
                    },
                );
            }

            let output_raw_due = if self.telemetry_rates.output_raw_hz == 0 {
                self.telemetry_rates.output_raw_imu_divisor != 0
                    && self.output_raw_imu_count % self.telemetry_rates.output_raw_imu_divisor == 0
            } else {
                stream_due(
                    now_us,
                    &mut self.telemetry_rate_state.output_raw_us,
                    self.telemetry_rates.output_raw_hz,
                )
            };
            if output_raw_due {
                self.send_output_raw(board, actuator_commands);
            }
            self.output_raw_imu_count = self.output_raw_imu_count.wrapping_add(1);
        }

        if let Some(packet) = processed_sensors.pitot {
            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.diff_pressure_us,
                self.telemetry_rates.diff_pressure_hz,
            ) {
                self.send_rosflight_diff_pressure(
                    board,
                    DiffPressureMsg {
                        velocity: packet.indicated_airspeed,
                        diff_pressure: packet.differential_pressure,
                        temperature: packet.temperature,
                    },
                );
            }
        }

        if let Some(packet) = processed_sensors.baro {
            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.baro_us,
                self.telemetry_rates.baro_hz,
            ) {
                self.send_rosflight_small_baro(
                    board,
                    SmallBaroMsg {
                        altitude: packet.altitude,
                        pressure: packet.pressure,
                        temperature: packet.temperature,
                    },
                );
            }
        }

        if let Some(packet) = processed_sensors.mag {
            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.mag_us,
                self.telemetry_rates.mag_hz,
            ) {
                self.send_rosflight_small_mag(
                    board,
                    SmallMagMsg {
                        xmag: packet.flux[0],
                        ymag: packet.flux[1],
                        zmag: packet.flux[2],
                    },
                );
            }
        }

        if let Some(packet) = processed_sensors.range {
            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.range_us,
                self.telemetry_rates.range_hz,
            ) {
                self.send_rosflight_small_range(
                    board,
                    SmallRangeMsg {
                        type_: match packet.range_type {
                            RangeType::Sonar => RosflightRangeType::RosflightRangeSonar,
                            RangeType::Lidar => RosflightRangeType::RosflightRangeLidar,
                        },
                        range: packet.range,
                        max_range: packet.max_range,
                        min_range: packet.min_range,
                    },
                );
            }
        }

        if let Some(packet) = processed_sensors.battery {
            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.battery_us,
                self.telemetry_rates.battery_hz,
            ) {
                self.send_rosflight_battery_status(
                    board,
                    BatteryStatusMsg {
                        battery_voltage: packet.voltage,
                        battery_current: packet.current,
                    },
                );
            }
        }

        if let Some(packet) = processed_sensors.gnss {
            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.gnss_us,
                self.telemetry_rates.gnss_hz,
            ) {
                self.send_rosflight_gnss(
                    board,
                    RosflightGnssMsg {
                        rosflight_timestamp: packet.header.timestamp,
                        seconds: packet.unix_seconds,
                        nanos: packet.unix_nanos,
                        fix_type: packet.fix_type,
                        num_sat: packet.num_sats,
                        lat: packet.lat,
                        lon: packet.lon,
                        height: packet.height,
                        vel_n: packet.vel_n,
                        vel_e: packet.vel_e,
                        vel_d: packet.vel_d,
                        s_acc: packet.s_acc,
                        h_acc: packet.h_acc,
                        v_acc: packet.v_acc,
                    },
                );
            }
        }

        if let Some(packet) = processed_sensors.rc {
            if stream_due(
                now_us,
                &mut self.telemetry_rate_state.rc_us,
                self.telemetry_rates.rc_hz,
            ) {
                let mut channels = [0u16; RC_PACKET_CHANNELS];
                let count = (packet.n_chan as usize).min(8).min(RC_PACKET_CHANNELS);
                for (dst, src) in channels.iter_mut().zip(packet.chan.iter()).take(count) {
                    *dst = (*src * 1000.0 + 1000.0) as u16;
                }

                self.send_rosflight_rc_raw(
                    board,
                    RcChannelsMsg {
                        time_boot_ms: board.clock_millis(),
                        chancount: count as u8,
                        channels,
                        rssi: 0,
                    },
                );
            }
        }
    }

    pub fn send_one_named_telemetry_stream<S, A, R>(
        &mut self,
        board: &mut B,
        now_us: u64,
        state_manager: &StateManager,
        command_manager: &CommandManager,
        params: &Params,
        estimator_state: &S,
        processed_sensors: &ProcessedSensors<R>,
        actuator_commands: &A,
        sensor_error_count: u16,
        loop_time_us: u16,
    ) -> bool
    where
        S: AttitudeEstimate,
        A: AsRef<[R]>,
        R: FlightFloat,
    {
        let Some(stream) = self.select_due_named_telemetry_stream(now_us, processed_sensors, true)
        else {
            return false;
        };
        #[cfg(feature = "timing-diagnostics")]
        record_telemetry_stream_selected(stream);

        self.send_selected_named_telemetry_stream(
            stream,
            board,
            now_us,
            state_manager,
            command_manager,
            params,
            estimator_state,
            processed_sensors,
            actuator_commands,
            sensor_error_count,
            loop_time_us,
        )
    }

    pub fn send_named_telemetry_stream_if_due<S, A, R>(
        &mut self,
        stream: NamedTelemetryStream,
        board: &mut B,
        now_us: u64,
        state_manager: &StateManager,
        command_manager: &CommandManager,
        params: &Params,
        estimator_state: &S,
        processed_sensors: &ProcessedSensors<R>,
        actuator_commands: &A,
        sensor_error_count: u16,
        loop_time_us: u16,
    ) -> bool
    where
        S: AttitudeEstimate,
        A: AsRef<[R]>,
        R: FlightFloat,
    {
        if self
            .named_telemetry_stream_deadline(stream, now_us, processed_sensors)
            .is_none()
        {
            return false;
        }
        #[cfg(feature = "timing-diagnostics")]
        {
            record_telemetry_stream_due(stream);
            record_telemetry_stream_selected(stream);
        }

        self.send_selected_named_telemetry_stream(
            stream,
            board,
            now_us,
            state_manager,
            command_manager,
            params,
            estimator_state,
            processed_sensors,
            actuator_commands,
            sensor_error_count,
            loop_time_us,
        )
    }

    fn send_selected_named_telemetry_stream<S, A, R>(
        &mut self,
        stream: NamedTelemetryStream,
        board: &mut B,
        now_us: u64,
        state_manager: &StateManager,
        command_manager: &CommandManager,
        params: &Params,
        estimator_state: &S,
        processed_sensors: &ProcessedSensors<R>,
        actuator_commands: &A,
        sensor_error_count: u16,
        loop_time_us: u16,
    ) -> bool
    where
        S: AttitudeEstimate,
        A: AsRef<[R]>,
        R: FlightFloat,
    {
        let sent = match stream {
            NamedTelemetryStream::Heartbeat => {
                self.send_rosflight_heartbeat(
                    board,
                    HeartbeatMsg {
                        autopilot: 0,
                        base_mode: 0,
                        custom_mode: 0,
                        mavlink_version: 0,
                        system_status: 0,
                        type_: if param_int(params, ParamId::PARAM_FIXED_WING) != 0 {
                            MAV_TYPE_FIXED_WING
                        } else {
                            MAV_TYPE_QUADROTOR
                        },
                    },
                );
                self.last_heartbeat_us = now_us;
                true
            }
            NamedTelemetryStream::Status => {
                self.send_rosflight_status(
                    board,
                    RosflightStatusMsg {
                        armed: state_manager.is_armed() as u8,
                        failsafe: state_manager.is_in_failsafe() as u8,
                        rc_override: command_manager.get_rc_override(),
                        offboard: command_manager.is_offboard_active() as u8,
                        error_code: state_manager.get_errors(),
                        control_mode: command_manager.get_control_mode().into(),
                        num_errors: sensor_error_count as i16,
                        loop_time_us: loop_time_us as i16,
                    },
                );
                self.last_status_send_us = now_us;
                true
            }
            NamedTelemetryStream::Imu => self.send_imu_if_due(board, now_us, processed_sensors),
            NamedTelemetryStream::Rc => self.send_rc_if_due(board, now_us, processed_sensors),
            NamedTelemetryStream::Attitude => match processed_sensors.imu {
                Some(imu_packet) => {
                    if !stream_due(
                        now_us,
                        &mut self.telemetry_rate_state.attitude_us,
                        self.telemetry_rates.attitude_hz,
                    ) {
                        false
                    } else {
                        let q = estimator_state.q();
                        let qd = estimator_state.q_dot();
                        let rollspeed =
                            2.0 * (q[0] * qd[1] - q[1] * qd[0] - q[2] * qd[3] + q[3] * qd[2]);
                        let pitchspeed =
                            2.0 * (q[0] * qd[2] - q[1] * qd[3] - q[2] * qd[0] + q[3] * qd[1]);
                        let yawspeed =
                            2.0 * (q[0] * qd[3] - q[1] * qd[2] - q[2] * qd[1] + q[3] * qd[0]);
                        self.send_rosflight_attitude_quaternion(
                            board,
                            AttitudeQuaternionMsg {
                                time_boot_ms: (imu_packet.header.timestamp / 1000) as u32,
                                q1: q[0],
                                q2: q[1],
                                q3: q[2],
                                q4: q[3],
                                rollspeed,
                                pitchspeed,
                                yawspeed,
                            },
                        );
                        true
                    }
                }
                None => false,
            },
            NamedTelemetryStream::OutputRaw => {
                if processed_sensors.imu.is_none() {
                    false
                } else {
                    let output_raw_due = if self.telemetry_rates.output_raw_hz == 0 {
                        self.telemetry_rates.output_raw_imu_divisor != 0
                            && self.output_raw_imu_count
                                % self.telemetry_rates.output_raw_imu_divisor
                                == 0
                    } else {
                        stream_due(
                            now_us,
                            &mut self.telemetry_rate_state.output_raw_us,
                            self.telemetry_rates.output_raw_hz,
                        )
                    };
                    self.output_raw_imu_count = self.output_raw_imu_count.wrapping_add(1);
                    if !output_raw_due {
                        false
                    } else {
                        self.send_output_raw(board, actuator_commands);
                        true
                    }
                }
            }
            NamedTelemetryStream::Gnss => match processed_sensors.gnss {
                Some(packet) => {
                    if !stream_due(
                        now_us,
                        &mut self.telemetry_rate_state.gnss_us,
                        self.telemetry_rates.gnss_hz,
                    ) {
                        false
                    } else {
                        self.send_rosflight_gnss(
                            board,
                            RosflightGnssMsg {
                                rosflight_timestamp: packet.header.timestamp,
                                seconds: packet.unix_seconds,
                                nanos: packet.unix_nanos,
                                fix_type: packet.fix_type,
                                num_sat: packet.num_sats,
                                lat: packet.lat,
                                lon: packet.lon,
                                height: packet.height,
                                vel_n: packet.vel_n,
                                vel_e: packet.vel_e,
                                vel_d: packet.vel_d,
                                s_acc: packet.s_acc,
                                h_acc: packet.h_acc,
                                v_acc: packet.v_acc,
                            },
                        );
                        true
                    }
                }
                None => false,
            },
            NamedTelemetryStream::DiffPressure => match processed_sensors.pitot {
                Some(packet) => {
                    if !stream_due(
                        now_us,
                        &mut self.telemetry_rate_state.diff_pressure_us,
                        self.telemetry_rates.diff_pressure_hz,
                    ) {
                        false
                    } else {
                        self.send_rosflight_diff_pressure(
                            board,
                            DiffPressureMsg {
                                velocity: packet.indicated_airspeed,
                                diff_pressure: packet.differential_pressure,
                                temperature: packet.temperature,
                            },
                        );
                        true
                    }
                }
                None => false,
            },
            NamedTelemetryStream::Baro => match processed_sensors.baro {
                Some(packet) => {
                    if !stream_due(
                        now_us,
                        &mut self.telemetry_rate_state.baro_us,
                        self.telemetry_rates.baro_hz,
                    ) {
                        false
                    } else {
                        self.send_rosflight_small_baro(
                            board,
                            SmallBaroMsg {
                                altitude: packet.altitude,
                                pressure: packet.pressure,
                                temperature: packet.temperature,
                            },
                        );
                        true
                    }
                }
                None => false,
            },
            NamedTelemetryStream::Mag => match processed_sensors.mag {
                Some(packet) => {
                    if !stream_due(
                        now_us,
                        &mut self.telemetry_rate_state.mag_us,
                        self.telemetry_rates.mag_hz,
                    ) {
                        false
                    } else {
                        self.send_rosflight_small_mag(
                            board,
                            SmallMagMsg {
                                xmag: packet.flux[0],
                                ymag: packet.flux[1],
                                zmag: packet.flux[2],
                            },
                        );
                        true
                    }
                }
                None => false,
            },
            NamedTelemetryStream::Range => match processed_sensors.range {
                Some(packet) => {
                    if !stream_due(
                        now_us,
                        &mut self.telemetry_rate_state.range_us,
                        self.telemetry_rates.range_hz,
                    ) {
                        false
                    } else {
                        self.send_rosflight_small_range(
                            board,
                            SmallRangeMsg {
                                type_: match packet.range_type {
                                    RangeType::Sonar => RosflightRangeType::RosflightRangeSonar,
                                    RangeType::Lidar => RosflightRangeType::RosflightRangeLidar,
                                },
                                range: packet.range,
                                max_range: packet.max_range,
                                min_range: packet.min_range,
                            },
                        );
                        true
                    }
                }
                None => false,
            },
            NamedTelemetryStream::Battery => match processed_sensors.battery {
                Some(packet) => {
                    if !stream_due(
                        now_us,
                        &mut self.telemetry_rate_state.battery_us,
                        self.telemetry_rates.battery_hz,
                    ) {
                        false
                    } else {
                        self.send_rosflight_battery_status(
                            board,
                            BatteryStatusMsg {
                                battery_voltage: packet.voltage,
                                battery_current: packet.current,
                            },
                        );
                        true
                    }
                }
                None => false,
            },
        };
        #[cfg(feature = "timing-diagnostics")]
        if sent {
            record_telemetry_stream_sent(stream);
        } else {
            record_telemetry_stream_failed(stream);
        }
        sent
    }

    #[cfg(feature = "timing-diagnostics")]
    pub fn telemetry_scheduler_diagnostic_text(&mut self) -> Option<[u8; 50]> {
        while self.telemetry_diag_text_index < NAMED_TELEMETRY_STREAM_COUNT {
            let index = self.telemetry_diag_text_index;
            self.telemetry_diag_text_index += 1;
            let Some(stream) = NamedTelemetryStream::from_index(index) else {
                continue;
            };
            let due = TELEMETRY_STREAM_DUE[index].swap(0, Ordering::Relaxed);
            let selected = TELEMETRY_STREAM_SELECTED[index].swap(0, Ordering::Relaxed);
            let sent = TELEMETRY_STREAM_SENT[index].swap(0, Ordering::Relaxed);
            let failed = TELEMETRY_STREAM_FAILED[index].swap(0, Ordering::Relaxed);
            if due == 0 && selected == 0 && sent == 0 && failed == 0 {
                continue;
            }
            let mut text = String::<50>::new();
            let _ = write!(
                text,
                "TMS {} d{} s{} ok{} f{}",
                stream.label(),
                due,
                selected,
                sent,
                failed
            );
            let mut bytes = [0_u8; 50];
            let payload = text.as_bytes();
            bytes[..payload.len()].copy_from_slice(payload);
            return Some(bytes);
        }
        self.telemetry_diag_text_index = 0;
        None
    }

    fn send_imu_if_due<R>(
        &mut self,
        board: &mut B,
        now_us: u64,
        processed_sensors: &ProcessedSensors<R>,
    ) -> bool
    where
        R: FlightFloat,
    {
        let Some(imu_packet) = processed_sensors.imu else {
            return false;
        };
        if self.last_realtime_imu_telemetry_timestamp == Some(imu_packet.header.timestamp) {
            return false;
        }
        if !stream_due(
            now_us,
            &mut self.telemetry_rate_state.imu_us,
            self.telemetry_rates.imu_hz,
        ) {
            return false;
        }
        self.send_rosflight_small_imu(
            board,
            SmallImuMsg {
                temperature: imu_packet.temperature,
                time_boot_us: imu_packet.header.timestamp,
                xacc: imu_packet.accel[0].to_f32_lossy(),
                yacc: imu_packet.accel[1].to_f32_lossy(),
                zacc: imu_packet.accel[2].to_f32_lossy(),
                xgyro: imu_packet.gyro[0].to_f32_lossy(),
                ygyro: imu_packet.gyro[1].to_f32_lossy(),
                zgyro: imu_packet.gyro[2].to_f32_lossy(),
            },
        );
        self.last_realtime_imu_telemetry_timestamp = Some(imu_packet.header.timestamp);
        true
    }

    fn send_rc_if_due<R>(
        &mut self,
        board: &mut B,
        now_us: u64,
        processed_sensors: &ProcessedSensors<R>,
    ) -> bool
    where
        R: FlightFloat,
    {
        let Some(packet) = processed_sensors.rc else {
            return false;
        };
        if !stream_due(
            now_us,
            &mut self.telemetry_rate_state.rc_us,
            self.telemetry_rates.rc_hz,
        ) {
            return false;
        }
        let mut channels = [0u16; RC_PACKET_CHANNELS];
        let count = (packet.n_chan as usize).min(8).min(RC_PACKET_CHANNELS);
        for (dst, src) in channels.iter_mut().zip(packet.chan.iter()).take(count) {
            *dst = (*src * 1000.0 + 1000.0) as u16;
        }
        self.send_rosflight_rc_raw(
            board,
            RcChannelsMsg {
                time_boot_ms: board.clock_millis(),
                chancount: count as u8,
                channels,
                rssi: 0,
            },
        );
        true
    }

    fn send_output_raw<A, R>(&mut self, board: &mut B, actuator_commands: &A)
    where
        A: AsRef<[R]>,
        R: FlightFloat,
    {
        let mut values = [0.0f32; 14];
        for (dst, src) in values.iter_mut().zip(actuator_commands.as_ref().iter()) {
            *dst = src.to_f32_lossy();
        }
        self.send_rosflight_output_raw(
            board,
            RosflightOutputRawMsg {
                stamp: board.clock_millis() as u64,
                values,
            },
        );
    }

    pub fn act_on_messages(
        &mut self,
        param_events: &mut ParamEventQueues,
        comm_events: &mut CommEventQueues,
        command_events: &mut CommandEventQueues,
        companion_events: &mut CompanionEventQueues,
        board: &mut B,
    ) {
        if let Some(msg) = self.msgs.heartbeat.take() {
            companion_events
                .heartbeats
                .push_or_log(CompanionHeartbeatReceived { msg }, "companion heartbeat");
        }

        // first check the param_request_list
        if let Some(msg) = self.msgs.param_request_read.take() {
            if self.targets_this_system(msg.target_system) {
                param_events.read_requests.push_or_log(
                    ParamReadRequested {
                        identifier: msg.param_identifier,
                    },
                    "param read request",
                );
            }
        }

        if let Some(msg) = self.msgs.param_request_list.take() {
            if self.targets_this_system(msg.target_system) {
                param_events
                    .list_requests
                    .push_or_log(ParamListRequested, "param list request");
            }
        }

        // next check for timesync messages
        let msg_opt: Option<TimesyncMsg> = self.msgs.timesync.take();
        if let Some(mut msg) = msg_opt {
            if msg.tc1 == 0 {
                msg.tc1 = (board.clock_micros() * 1000) as i64;
                self.send_timesync(board, msg);
            }
        }

        if let Some(msg) = self.msgs.offboard_control.take() {
            let now_us = board.clock_micros();
            command_events
                .offboard_control_requests
                .push_or_log(OffboardControlRequested { now_us, msg }, "offboard control");
        }

        if let Some(msg) = self.msgs.aux_cmd.take() {
            companion_events
                .aux_commands
                .push_or_log(AuxCommandReceived { msg }, "aux command");
        }

        if let Some(msg) = self.msgs.external_attitude.take() {
            companion_events
                .external_attitudes
                .push_or_log(ExternalAttitudeReceived { msg }, "external attitude");
        }

        while !param_events.set_requests.is_full() {
            let Some(msg) = Store::<ParamSetMsg>::take(&mut self.msgs) else {
                break;
            };

            if self.targets_this_system(msg.target_system) {
                let pushed = param_events.set_requests.push_or_log(
                    ParamSetRequested {
                        value: msg.param_value,
                        param_id_bytes: msg.param_id,
                    },
                    "param set request",
                );
                debug_assert!(pushed);
            }
        }

        // now act on ROSflight Commands

        let cmd_msg_opt = self.msgs.cmd.take();
        if let Some(msg) = cmd_msg_opt {
            // Assume failure unless explicitly set to success
            let success = RosflightCmdResponse::RosflightCmdFailed;
            let mut send_ack_now = true;

            match msg.command {
                RosflightCmd::RcCalibration => {
                    if command_events.rc_trim_calibration_requests.push_or_log(
                        RcTrimCalibrationRequested {
                            command: msg.command,
                        },
                        "rc trim calibration",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::AccelCalibration => {
                    if command_events.calibration_requests.push_or_log(
                        CalibrationRequested {
                            command: msg.command,
                        },
                        "accel calibration",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::GyroCalibration => {
                    if command_events.calibration_requests.push_or_log(
                        CalibrationRequested {
                            command: msg.command,
                        },
                        "gyro calibration",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::BaroCalibration => {
                    if command_events.calibration_requests.push_or_log(
                        CalibrationRequested {
                            command: msg.command,
                        },
                        "baro calibration",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::AirspeedCalibration => {
                    if command_events.calibration_requests.push_or_log(
                        CalibrationRequested {
                            command: msg.command,
                        },
                        "airspeed calibration",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::ReadParams => {
                    if command_events.board_command_requests.push_or_log(
                        BoardCommandRequested {
                            command: msg.command,
                        },
                        "read params command",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::WriteParams => {
                    if command_events.board_command_requests.push_or_log(
                        BoardCommandRequested {
                            command: msg.command,
                        },
                        "write params command",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::SetParamDefaults => {
                    if command_events.param_defaults_requests.push_or_log(
                        ParamDefaultsRequested {
                            command: msg.command,
                        },
                        "param defaults command",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::Reboot => {
                    if command_events.board_command_requests.push_or_log(
                        BoardCommandRequested {
                            command: msg.command,
                        },
                        "reboot command",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::RebootToBootloader => {
                    if command_events.board_command_requests.push_or_log(
                        BoardCommandRequested {
                            command: msg.command,
                        },
                        "bootloader command",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::SendVersion => {
                    if command_events.version_requests.push_or_log(
                        VersionRequested {
                            command: msg.command,
                        },
                        "version command",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::ResetOrigin => {
                    if command_events.reset_origin_requests.push_or_log(
                        ResetOriginRequested {
                            command: msg.command,
                        },
                        "reset origin command",
                    ) {
                        send_ack_now = false;
                    }
                }
                RosflightCmd::SendAllConfigInfos => {
                    if command_events.config_info_requests.push_or_log(
                        ConfigInfoRequested {
                            command: msg.command,
                        },
                        "config info command",
                    ) {
                        send_ack_now = false;
                    }
                }
            } // end match

            if send_ack_now {
                let ack_msg = RosflightCmdAckMsg {
                    command: msg.command,
                    success,
                };
                comm_events
                    .responses
                    .push_or_log(CommResponse::CmdAck(ack_msg), "command ack response");
            }
        } // end if let Some(msg)
    }

    pub fn send_comm_responses(&mut self, board: &mut B, comm_events: &mut CommEventQueues) {
        self.send_comm_responses_limited(board, comm_events, usize::MAX);
    }

    pub fn send_comm_responses_limited(
        &mut self,
        board: &mut B,
        comm_events: &mut CommEventQueues,
        max_responses: usize,
    ) -> usize {
        let mut sent = 0;
        while sent < max_responses
            && let Some(response) = comm_events.responses.pop()
        {
            match response {
                CommResponse::ParamValue(msg) => {
                    if msg.param_index == ParamId::PARAM_SYSTEM_ID as u16 {
                        if let ParamValue::Int(new_sysid) = msg.param_value {
                            self.sysid = new_sysid as u8;
                        }
                    }
                    self.comm_link.send_named_value(board, self.sysid, msg);
                }
                CommResponse::CmdAck(msg) => {
                    self.comm_link.send_cmd_ack(board, self.sysid, msg);
                }
                CommResponse::Version(msg) => {
                    self.comm_link.send_version(board, self.sysid, msg);
                }
                CommResponse::Statustext(msg) => {
                    self.comm_link.send_statustext(board, self.sysid, msg);
                }
                CommResponse::HardError(msg) => {
                    self.comm_link.send_hard_error(board, self.sysid, msg);
                }
            }
            sent += 1;
        }
        sent
    }

    pub fn send_timesync(&mut self, board: &mut B, msg: TimesyncMsg) {
        self.comm_link.send_timesync(board, self.sysid, msg);
    }

    pub fn send_rosflight_heartbeat(&mut self, board: &mut B, msg: HeartbeatMsg) {
        self.comm_link.send_heartbeat(board, self.sysid, msg);
    }

    pub fn send_rosflight_status(&mut self, board: &mut B, msg: RosflightStatusMsg) {
        self.comm_link.send_status(board, self.sysid, msg);
    }

    pub fn send_rosflight_attitude_quaternion(
        &mut self,
        board: &mut B,
        msg: AttitudeQuaternionMsg,
    ) {
        self.comm_link.send_attitude(board, self.sysid, msg);
    }

    pub fn send_rosflight_small_imu(&mut self, board: &mut B, msg: SmallImuMsg) {
        self.comm_link.send_imu(board, self.sysid, msg);
    }

    pub fn send_rosflight_small_baro(&mut self, board: &mut B, msg: SmallBaroMsg) {
        self.comm_link.send_baro(board, self.sysid, msg);
    }

    pub fn send_rosflight_diff_pressure(&mut self, board: &mut B, msg: DiffPressureMsg) {
        self.comm_link.send_diff_pressure(board, self.sysid, msg);
    }

    pub fn send_rosflight_small_mag(&mut self, board: &mut B, msg: SmallMagMsg) {
        self.comm_link.send_mag(board, self.sysid, msg);
    }

    pub fn send_rosflight_small_range(&mut self, board: &mut B, msg: SmallRangeMsg) {
        self.comm_link.send_range(board, self.sysid, msg);
    }

    pub fn send_rosflight_battery_status(&mut self, board: &mut B, msg: BatteryStatusMsg) {
        self.comm_link.send_battery_status(board, self.sysid, msg);
    }

    pub fn send_rosflight_gnss(&mut self, board: &mut B, msg: RosflightGnssMsg) {
        self.comm_link.send_gnss(board, self.sysid, msg);
    }

    pub fn send_rosflight_rc_raw(&mut self, board: &mut B, msg: RcChannelsMsg) {
        self.comm_link.send_rc_raw(board, self.sysid, msg);
    }

    pub fn send_rosflight_output_raw(&mut self, board: &mut B, msg: RosflightOutputRawMsg) {
        self.comm_link.send_output_raw(board, self.sysid, msg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        board::BoardIo,
        comm::messages::{
            enums::{
                OffboardControlIgnore, OffboardControlMode, ParamIdentifier, RosflightAuxCmdType,
                RosflightCmd, RosflightCmdResponse,
            },
            messages::{
                ExternalAttitudeMsg, HeartbeatMsg, OffboardControlMsg, ParamRequestListMsg,
                ParamRequestReadMsg, ParamSetMsg, RosflightAuxCmdMsg, RosflightCmdMsg, TimesyncMsg,
            },
        },
        command::CommandManager,
        command::service::{self as command_service, CalibrationRequestCtx},
        events::{
            CommEventQueues, CommResponse, CommandEventQueues, CompanionEventQueues,
            ParamEventQueues,
        },
        params::service::{
            self as param_service, ParamApplyCtx, ParamListCtx, ParamListState, ParamReadCtx,
        },
        params::{ParamId, ParamValue, Params},
        ports::{EventDrainPort, EventEmitPort, ParamsReadPort, ParamsWritePort},
        sensors::ProcessedSensors,
        sensors::processors::CalibrationFlags,
        state_machine::{Event, StateManager},
        test_support::{RecordingCommLink, TestBoard},
    };

    fn initialized_state() -> StateManager {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state
    }

    fn companion_events() -> CompanionEventQueues {
        CompanionEventQueues::default()
    }

    #[test]
    fn stream_due_preserves_deadline_cadence_after_late_send() {
        let mut last_us = 0;

        assert!(stream_due(1_000, &mut last_us, 400));
        assert_eq!(last_us, 1_000);

        assert!(!stream_due(3_400, &mut last_us, 400));
        assert_eq!(last_us, 1_000);

        assert!(stream_due(3_700, &mut last_us, 400));
        assert_eq!(last_us, 3_500);

        assert!(!stream_due(5_900, &mut last_us, 400));
        assert_eq!(last_us, 3_500);

        assert!(stream_due(6_100, &mut last_us, 400));
        assert_eq!(last_us, 6_000);

        assert!(stream_due(13_900, &mut last_us, 400));
        assert_eq!(last_us, 13_500);
    }

    #[test]
    fn param_set_emits_request_without_mutating_or_acknowledging() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let params = Params::new();
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.store(ParamSetMsg {
            target_system: 1,
            target_component: 1,
            param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
            param_value: ParamValue::Int(42),
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert_eq!(manager.comm_link.sent_param_value_count, 0);

        let request = param_events.set_requests.pop().unwrap();
        assert_eq!(request.value, ParamValue::Int(42));
        assert_eq!(request.param_id_bytes, *b"SYS_ID\0\0\0\0\0\0\0\0\0\0");
    }

    #[test]
    fn param_set_ingress_waits_when_ecs_queue_is_full() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        for value in 10..15 {
            manager.msgs.store(ParamSetMsg {
                target_system: 1,
                target_component: 1,
                param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
                param_value: ParamValue::Int(value),
            });
        }

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert!(param_events.set_requests.is_full());
        assert_eq!(param_events.set_requests.len(), 4);
        assert_eq!(manager.msgs.param_set.len(), 1);

        let _ = param_events.set_requests.pop();
        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert!(param_events.set_requests.is_full());
        assert_eq!(manager.msgs.param_set.len(), 0);
        let values: heapless::Vec<_, 4> = param_events
            .set_requests
            .iter()
            .map(|req| req.value)
            .collect();
        assert_eq!(
            values.as_slice(),
            &[
                ParamValue::Int(11),
                ParamValue::Int(12),
                ParamValue::Int(13),
                ParamValue::Int(14),
            ]
        );
    }

    #[test]
    fn param_messages_for_other_system_are_ignored() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.store(ParamSetMsg {
            target_system: 42,
            target_component: 1,
            param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
            param_value: ParamValue::Int(42),
        });
        manager.msgs.param_request_list = Some(ParamRequestListMsg {
            target_system: 42,
            target_component: 1,
        });
        manager.msgs.param_request_read = Some(ParamRequestReadMsg {
            target_system: 42,
            target_component: 1,
            param_identifier: ParamIdentifier::ID(*b"SYS_ID\0\0\0\0\0\0\0\0\0\0"),
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert!(param_events.set_requests.is_empty());
        assert!(param_events.list_requests.is_empty());
        assert!(param_events.read_requests.is_empty());
    }

    #[test]
    fn param_request_list_emits_request_without_streaming_from_comms() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let params = Params::new();
        let mut param_list_state = ParamListState::default();
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.param_request_list = Some(ParamRequestListMsg {
            target_system: 1,
            target_component: 1,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link.sent_param_value_count, 0);
        assert_eq!(param_events.list_requests.len(), 1);

        param_service::service_param_list_requests(ParamListCtx {
            params: ParamsReadPort::new(&params),
            state: &mut param_list_state,
            requests: EventDrainPort::new(&mut param_events.list_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
        });

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.comm_link.sent_param_value_count, 1);
        let sent = manager.comm_link.sent_param_values[0].unwrap();
        assert_eq!(sent.param_index, ParamId::PARAM_BAUD_RATE as u16);
        assert_eq!(sent.param_value, ParamValue::Int(921600));
    }

    #[test]
    fn param_request_read_emits_request_without_reading_from_comms() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.param_request_read = Some(ParamRequestReadMsg {
            target_system: 1,
            target_component: 1,
            param_identifier: ParamIdentifier::ID(*b"SYS_ID\0\0\0\0\0\0\0\0\0\0"),
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link.sent_param_value_count, 0);
        assert_eq!(param_events.read_requests.len(), 1);

        param_service::service_param_read_requests(ParamReadCtx {
            params: ParamsReadPort::new(&params),
            requests: EventDrainPort::new(&mut param_events.read_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
        });

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.comm_link.sent_param_value_count, 1);
        let sent = manager.comm_link.sent_param_values[0].unwrap();
        assert_eq!(sent.param_index, ParamId::PARAM_SYSTEM_ID as u16);
        assert_eq!(sent.param_value, ParamValue::Int(42));
    }

    #[test]
    fn send_comm_responses_sends_param_value_and_updates_sysid() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut comm_events = CommEventQueues::default();

        let _ = comm_events
            .responses
            .push(CommResponse::ParamValue(ParamValueMsg {
                param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
                param_value: ParamValue::Int(42),
                param_count: 1,
                param_index: ParamId::PARAM_SYSTEM_ID as u16,
            }));

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.sysid, 42);
        assert_eq!(manager.comm_link.sent_param_value_count, 1);

        let sent = manager.comm_link.sent_param_values[0].unwrap();
        assert_eq!(sent.param_id, *b"SYS_ID\0\0\0\0\0\0\0\0\0\0");
        assert_eq!(sent.param_value, ParamValue::Int(42));
        assert!(comm_events.responses.is_empty());
    }

    #[test]
    fn send_comm_responses_sends_command_ack_and_version() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut comm_events = CommEventQueues::default();

        let _ = comm_events
            .responses
            .push(CommResponse::Version(RosflightVersionMsg {
                version: [7; 50],
            }));
        let _ = comm_events
            .responses
            .push(CommResponse::CmdAck(RosflightCmdAckMsg {
                command: RosflightCmd::SendVersion,
                success: RosflightCmdResponse::RosflightCmdSuccess,
            }));
        let _ = comm_events
            .responses
            .push(CommResponse::Statustext(StatustextMsg {
                severity: Severity::Info,
                text: [9; 50],
            }));

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.comm_link().version_count, 1);
        assert_eq!(manager.comm_link().last_version.unwrap().version, [7; 50]);
        assert_eq!(manager.comm_link().cmd_ack_count, 1);
        assert_eq!(manager.comm_link().statustext_count, 1);
        assert_eq!(manager.comm_link().last_statustext.unwrap().text, [9; 50]);
        assert!(matches!(
            manager.comm_link().last_cmd_ack.unwrap().command,
            RosflightCmd::SendVersion
        ));
        assert!(comm_events.responses.is_empty());
    }

    #[test]
    fn send_version_command_enqueues_version_and_ack_responses() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SendVersion,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().version_count, 0);
        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());
        assert_eq!(command_events.version_requests.len(), 1);

        command_service::apply_version_requests(command_service::VersionRequestCtx {
            requests: EventDrainPort::new(&mut command_events.version_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
            state: &initialized_state(),
        });
        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.comm_link().version_count, 1);
        assert_eq!(manager.comm_link().cmd_ack_count, 1);
        assert!(matches!(
            manager.comm_link().last_cmd_ack.unwrap().success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn param_set_pipeline_defers_ack_until_after_apply_stage() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut params = Params::new();
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.store(ParamSetMsg {
            target_system: 1,
            target_component: 1,
            param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
            param_value: ParamValue::Int(42),
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert_eq!(manager.comm_link.sent_param_value_count, 0);

        param_service::apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut params),
            requests: EventDrainPort::new(&mut param_events.set_requests),
            changes: EventEmitPort::new(&mut param_events.changes),
            responses: EventEmitPort::new(&mut comm_events.responses),
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(manager.comm_link.sent_param_value_count, 0);

        let change = param_events.changes.iter().next().unwrap();
        assert_eq!(change.id, ParamId::PARAM_SYSTEM_ID);
        assert_eq!(change.old, ParamValue::Int(1));
        assert_eq!(change.new, ParamValue::Int(42));

        manager.send_comm_responses(&mut board, &mut comm_events);

        assert_eq!(manager.sysid, 42);
        assert_eq!(manager.comm_link.sent_param_value_count, 1);
        assert!(comm_events.responses.is_empty());
    }

    #[test]
    fn named_telemetry_sends_sensor_state_and_output_messages() {
        let mut board = TestBoard {
            current_time_us: 1_100_000,
            tx_write_count: 0,
            ..Default::default()
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), 0);
        let state_manager = StateManager::new();
        let command_manager = CommandManager::new();
        let params = Params::new();
        let estimator_state = crate::estimator::quad::AttitudeState::<f64>::default();
        let actuator_commands = [0.1, 0.2, 0.3, 0.4];
        let mut processed_sensors = ProcessedSensors::<f64>::default();
        processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 9_000,
                status: 0,
            },
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            temperature: 25.0,
            seq: 1,
        });
        processed_sensors.pitot = Some(crate::packets::PitotPacket {
            differential_pressure: 12.5,
            indicated_airspeed: 8.25,
            temperature: 24.0,
            ..Default::default()
        });
        processed_sensors.baro = Some(crate::packets::BaroPacket {
            altitude: 123.0,
            pressure: 95_000.0,
            temperature: 22.0,
            ..Default::default()
        });
        processed_sensors.range = Some(crate::packets::RangePacket {
            range: 3.5,
            min_range: 0.25,
            max_range: 8.0,
            range_type: crate::packets::RangeType::Lidar,
            ..Default::default()
        });
        processed_sensors.gnss = Some(crate::packets::GNSSPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 77_000,
                status: 0,
            },
            unix_seconds: 1_700_000_001,
            unix_nanos: 123_456_789,
            num_sats: 9,
            ..Default::default()
        });

        let now_us = board.clock_micros();

        manager.send_named_telemetry_streams(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        );

        assert_eq!(manager.comm_link().heartbeat_count, 1);
        assert_eq!(manager.comm_link().status_count, 1);
        assert_eq!(manager.comm_link().imu_count, 1);
        assert_eq!(manager.comm_link().attitude_count, 1);
        assert_eq!(manager.comm_link().diff_pressure_count, 1);
        assert_eq!(manager.comm_link().baro_count, 1);
        assert_eq!(manager.comm_link().range_count, 1);
        assert_eq!(manager.comm_link().gnss_count, 1);
        assert_eq!(manager.comm_link().output_raw_count, 1);
        assert_eq!(manager.comm_link().last_imu.unwrap().temperature, 25.0);
        assert_eq!(
            manager.comm_link().last_diff_pressure.unwrap().velocity,
            8.25
        );
        assert_eq!(manager.comm_link().last_baro.unwrap().altitude, 123.0);
        let range = manager.comm_link().last_range.unwrap();
        assert!(matches!(
            range.type_,
            RosflightRangeType::RosflightRangeLidar
        ));
        assert_eq!(range.min_range, 0.25);
        assert_eq!(range.max_range, 8.0);
        let gnss = manager.comm_link().last_gnss.unwrap();
        assert_eq!(gnss.seconds, 1_700_000_001);
        assert_eq!(gnss.nanos, 123_456_789);

        let output = manager.comm_link().last_output_raw.unwrap();
        assert_eq!(output.stamp, 1100);
        assert_eq!(output.values[0], 0.1);
        assert_eq!(output.values[1], 0.2);
        assert_eq!(output.values[2], 0.3);
        assert_eq!(output.values[3], 0.4);
    }

    #[test]
    fn realtime_named_telemetry_sends_deadline_ordered_streams() {
        let mut board = TestBoard {
            current_time_us: 1_100_000,
            tx_write_count: 0,
            ..Default::default()
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        manager.set_telemetry_rates(TelemetryRates::bounded_high_rate_transport());
        let state_manager = StateManager::new();
        let command_manager = CommandManager::new();
        let params = Params::new();
        let estimator_state = crate::estimator::quad::AttitudeState::<f64>::default();
        let actuator_commands = [0.1, 0.2, 0.3, 0.4];
        let mut processed_sensors = ProcessedSensors::<f64>::default();
        processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 9_000,
                status: 0,
            },
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            temperature: 25.0,
            seq: 1,
        });
        processed_sensors.baro = Some(crate::packets::BaroPacket {
            altitude: 123.0,
            pressure: 95_000.0,
            temperature: 22.0,
            ..Default::default()
        });

        let now_us = board.clock_micros();
        assert!(manager.send_one_named_telemetry_stream(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        ));
        assert_eq!(manager.comm_link().imu_count, 1);
        assert_eq!(manager.comm_link().attitude_count, 0);
        assert_eq!(manager.comm_link().output_raw_count, 0);
        assert_eq!(manager.comm_link().baro_count, 0);

        assert!(manager.send_one_named_telemetry_stream(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        ));
        assert_eq!(manager.comm_link().imu_count, 1);
        assert_eq!(manager.comm_link().attitude_count, 1);
        assert_eq!(manager.comm_link().output_raw_count, 0);
        assert_eq!(manager.comm_link().baro_count, 0);

        assert!(manager.send_one_named_telemetry_stream(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        ));
        assert_eq!(manager.comm_link().output_raw_count, 1);
        assert_eq!(manager.comm_link().baro_count, 0);

        assert!(manager.send_one_named_telemetry_stream(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        ));
        assert_eq!(manager.comm_link().output_raw_count, 1);
        assert_eq!(manager.comm_link().baro_count, 1);
    }

    #[test]
    fn realtime_priority_telemetry_uses_stream_due_and_freshness_gates() {
        let mut board = TestBoard {
            current_time_us: 1_100_000,
            tx_write_count: 0,
            ..Default::default()
        };
        let now_us = board.clock_micros();
        let mut manager = CommManager::new(RecordingCommLink::new(), now_us);
        manager.set_telemetry_rates(TelemetryRates::bounded_high_rate_transport());
        manager.last_heartbeat_us = now_us;
        manager.last_status_send_us = now_us;
        manager.telemetry_rate_state.rc_us = 1_000_000;
        manager.telemetry_rate_state.imu_us = 1_097_500;

        let state_manager = StateManager::new();
        let command_manager = CommandManager::new();
        let params = Params::new();
        let estimator_state = crate::estimator::quad::AttitudeState::<f64>::default();
        let actuator_commands = [0.1, 0.2, 0.3, 0.4];
        let mut processed_sensors = ProcessedSensors::<f64>::default();
        processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 9_000,
                status: 0,
            },
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            temperature: 25.0,
            seq: 1,
        });
        processed_sensors.rc = Some(crate::packets::RcPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 8_000,
                status: 0,
            },
            n_chan: 8,
            chan: [0.5; RC_PACKET_CHANNELS],
            lol: false,
        });

        assert!(manager.send_named_telemetry_stream_if_due(
            NamedTelemetryStream::Imu,
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        ));
        assert_eq!(manager.comm_link().imu_count, 1);
        assert_eq!(manager.comm_link().rc_channels_count, 0);

        assert!(!manager.send_named_telemetry_stream_if_due(
            NamedTelemetryStream::Imu,
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        ));
        assert_eq!(manager.comm_link().imu_count, 1);

        assert!(manager.send_one_named_telemetry_stream(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        ));
        assert_eq!(manager.comm_link().imu_count, 1);
    }

    #[test]
    fn named_telemetry_matches_upstream_output_raw_every_eighth_imu_sample() {
        let mut board = TestBoard {
            current_time_us: 1_100_000,
            tx_write_count: 0,
            ..Default::default()
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), 0);
        let state_manager = StateManager::new();
        let command_manager = CommandManager::new();
        let params = Params::new();
        let estimator_state = crate::estimator::quad::AttitudeState::<f64>::default();
        let mut processed_sensors = ProcessedSensors::<f64>::default();
        processed_sensors.imu = Some(crate::packets::ImuPacket::default());

        for i in 0..8 {
            board.current_time_us = 1_100_000 + i;
            let now_us = board.clock_micros();
            manager.send_named_telemetry_streams(
                &mut board,
                now_us,
                &state_manager,
                &command_manager,
                &params,
                &estimator_state,
                &processed_sensors,
                &[0.0; 4],
                0,
                0,
            );
        }

        assert_eq!(manager.comm_link().imu_count, 8);
        assert_eq!(manager.comm_link().attitude_count, 8);
        assert_eq!(manager.comm_link().output_raw_count, 1);

        board.current_time_us += 1;
        let now_us = board.clock_micros();
        manager.send_named_telemetry_streams(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &[0.0; 4],
            0,
            0,
        );

        assert_eq!(manager.comm_link().output_raw_count, 2);
    }

    #[test]
    fn explicit_telemetry_rates_bound_high_rate_streams() {
        let mut board = TestBoard {
            current_time_us: 1_000,
            tx_write_count: 0,
            ..Default::default()
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), 0);
        manager.set_telemetry_rates(TelemetryRates::bounded_high_rate_transport());
        let state_manager = StateManager::new();
        let command_manager = CommandManager::new();
        let params = Params::new();
        let estimator_state = crate::estimator::quad::AttitudeState::<f64>::default();
        let mut processed_sensors = ProcessedSensors::<f64>::default();
        processed_sensors.imu = Some(crate::packets::ImuPacket::default());
        processed_sensors.baro = Some(crate::packets::BaroPacket::default());

        let send_at = |manager: &mut CommManager<TestBoard, RecordingCommLink>,
                       board: &mut TestBoard,
                       now_us: u64| {
            board.current_time_us = now_us;
            manager.send_named_telemetry_streams(
                board,
                now_us,
                &state_manager,
                &command_manager,
                &params,
                &estimator_state,
                &processed_sensors,
                &[0.0; 4],
                0,
                0,
            );
        };

        send_at(&mut manager, &mut board, 1_000);
        assert_eq!(manager.comm_link().imu_count, 1);
        assert_eq!(manager.comm_link().attitude_count, 1);
        assert_eq!(manager.comm_link().baro_count, 1);
        assert_eq!(manager.comm_link().output_raw_count, 1);

        send_at(&mut manager, &mut board, 2_000);
        assert_eq!(manager.comm_link().imu_count, 1);
        assert_eq!(manager.comm_link().attitude_count, 1);
        assert_eq!(manager.comm_link().baro_count, 1);
        assert_eq!(manager.comm_link().output_raw_count, 1);

        send_at(&mut manager, &mut board, 3_500);
        assert_eq!(manager.comm_link().imu_count, 2);
        assert_eq!(manager.comm_link().attitude_count, 1);
        assert_eq!(manager.comm_link().baro_count, 1);
        assert_eq!(manager.comm_link().output_raw_count, 1);

        send_at(&mut manager, &mut board, 11_000);
        assert_eq!(manager.comm_link().imu_count, 3);
        assert_eq!(manager.comm_link().attitude_count, 1);
        assert_eq!(manager.comm_link().baro_count, 1);
        assert_eq!(manager.comm_link().output_raw_count, 1);

        send_at(&mut manager, &mut board, 21_000);
        assert_eq!(manager.comm_link().imu_count, 4);
        assert_eq!(manager.comm_link().attitude_count, 2);
        assert_eq!(manager.comm_link().baro_count, 1);
        assert_eq!(manager.comm_link().output_raw_count, 2);

        send_at(&mut manager, &mut board, 41_000);
        assert_eq!(manager.comm_link().imu_count, 5);
        assert_eq!(manager.comm_link().attitude_count, 3);
        assert_eq!(manager.comm_link().baro_count, 2);
        assert_eq!(manager.comm_link().output_raw_count, 3);
    }

    #[test]
    fn named_rc_telemetry_matches_upstream_raw_channel_packing() {
        let mut board = TestBoard {
            current_time_us: 1_234_000,
            tx_write_count: 0,
            ..Default::default()
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), 0);
        let state_manager = StateManager::new();
        let command_manager = CommandManager::new();
        let params = Params::new();
        let estimator_state = crate::estimator::quad::AttitudeState::<f64>::default();
        let mut processed_sensors = ProcessedSensors::<f64>::default();
        let mut rc_packet = crate::packets::RcPacket::default();
        rc_packet.n_chan = RC_PACKET_CHANNELS as u32;
        let test_channels = [
            -1.0, -0.5, 0.0, 0.5, 1.0, 0.25, -0.25, 0.75, 0.33, 0.44, 0.55, 0.66, 0.77, 0.88, 0.99,
            -0.99, 1.0, -1.0,
        ];
        rc_packet.chan[..test_channels.len()].copy_from_slice(&test_channels);
        processed_sensors.rc = Some(rc_packet);
        let now_us = board.clock_micros();

        manager.send_named_telemetry_streams(
            &mut board,
            now_us,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &[0.0; 4],
            0,
            0,
        );

        let msg = manager.comm_link().last_rc_channels.unwrap();
        assert_eq!(manager.comm_link().rc_channels_count, 1);
        assert_eq!(msg.time_boot_ms, 1234);
        assert_eq!(msg.chancount, 8);
        assert_eq!(msg.rssi, 0);
        assert_eq!(
            &msg.channels[..8],
            &[0, 500, 1000, 1500, 2000, 1250, 750, 1750]
        );
        assert!(msg.channels[8..].iter().all(|channel| *channel == 0));
    }

    #[test]
    fn named_status_telemetry_reports_command_manager_override_state() {
        let mut board = TestBoard {
            current_time_us: 1_100_000,
            tx_write_count: 0,
            ..Default::default()
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), 0);
        let state_manager = StateManager::new();
        let mut command_manager = CommandManager::new();
        let params = Params::new();
        let estimator_state = crate::estimator::quad::AttitudeState::<f64>::default();
        let processed_sensors = ProcessedSensors::<f64>::default();
        let actuator_commands = [0.0, 0.0, 0.0, 0.0];

        command_manager.set_new_offboard_command(
            board.clock_micros(),
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: 0.0,
                passthrough: [0.0; 4],
            },
            &params,
        );

        manager.send_named_telemetry_streams(
            &mut board,
            1_100_000,
            &state_manager,
            &command_manager,
            &params,
            &estimator_state,
            &processed_sensors,
            &actuator_commands,
            0,
            0,
        );

        let status = manager.comm_link().last_status.unwrap();
        assert_eq!(status.offboard, 1);
        assert_eq!(status.rc_override, 0);
    }

    #[test]
    fn calibration_command_ack_is_sent_when_calibration_starts() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();
        let mut cal_flags = CalibrationFlags::empty();
        let mut params = Params::new();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::GyroCalibration,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert!(cal_flags.is_empty());
        command_service::apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut command_events.calibration_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
            state: &initialized_state(),
            flags: &mut cal_flags,
            params: &mut params,
        });

        assert!(cal_flags.contains(CalibrationFlags::GYRO));
        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        manager.send_comm_responses(&mut board, &mut comm_events);
        assert_eq!(manager.comm_link().cmd_ack_count, 1);

        let ack = manager.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::GyroCalibration));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn offboard_control_message_emits_command_event() {
        let mut board = TestBoard {
            current_time_us: 55_000,
            tx_write_count: 0,
            ..Default::default()
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.offboard_control = Some(OffboardControlMsg {
            mode: OffboardControlMode::ModeRollPitchYawrateThrottle,
            ignore: OffboardControlIgnore::IGNORE_FY,
            qx: 0.1,
            qy: 0.2,
            qz: 0.3,
            fx: 0.4,
            fy: 0.5,
            fz: 0.6,
            passthrough: [0.0; 4],
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        let request = command_events.offboard_control_requests.pop().unwrap();
        assert_eq!(request.now_us, 55_000);
        assert_eq!(
            request.msg.mode,
            OffboardControlMode::ModeRollPitchYawrateThrottle
        );
        assert!(
            request
                .msg
                .ignore
                .contains(OffboardControlIgnore::IGNORE_FY)
        );
        assert_eq!(request.msg.qx, 0.1);
    }

    #[test]
    fn companion_inputs_emit_companion_events() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();
        let mut companion_events = CompanionEventQueues::default();

        manager.msgs.heartbeat = Some(HeartbeatMsg {
            type_: 1,
            autopilot: 2,
            base_mode: 3,
            custom_mode: 4,
            system_status: 5,
            mavlink_version: 6,
        });
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; 14],
            aux_cmd_array: [0.0; 14],
        };
        aux.type_array[1] = RosflightAuxCmdType::Motor;
        aux.aux_cmd_array[1] = 0.4;
        manager.msgs.aux_cmd = Some(aux);
        manager.msgs.external_attitude = Some(ExternalAttitudeMsg {
            qw: 1.0,
            qx: 0.1,
            qy: 0.2,
            qz: 0.3,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events,
            &mut board,
        );

        assert_eq!(companion_events.heartbeats.len(), 1);
        assert_eq!(companion_events.aux_commands.len(), 1);
        assert_eq!(companion_events.external_attitudes.len(), 1);
        assert_eq!(
            companion_events.heartbeats.pop().unwrap().msg.system_status,
            5
        );
        let aux_event = companion_events.aux_commands.pop().unwrap();
        assert!(matches!(
            aux_event.msg.type_array[1],
            RosflightAuxCmdType::Motor
        ));
        assert_eq!(aux_event.msg.aux_cmd_array[1], 0.4);
        assert_eq!(
            companion_events.external_attitudes.pop().unwrap().msg.qz,
            0.3
        );
    }

    #[test]
    fn timesync_responds_only_to_requests_and_uses_local_time() {
        let mut board = TestBoard {
            current_time_us: 123,
            ..Default::default()
        };
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.timesync = Some(TimesyncMsg { tc1: 99, ts1: 55 });
        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );
        assert_eq!(manager.comm_link().timesync_count, 0);

        manager.msgs.timesync = Some(TimesyncMsg { tc1: 0, ts1: 55 });
        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().timesync_count, 1);
        let response = manager.comm_link().last_timesync.unwrap();
        assert_eq!(response.tc1, 123_000);
        assert_eq!(response.ts1, 55);
    }

    #[test]
    fn set_param_defaults_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SetParamDefaults,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(manager.comm_link().cmd_ack_count, 0);

        command_service::apply_param_defaults_requests(command_service::ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut command_events.param_defaults_requests),
            responses: EventEmitPort::new(&mut comm_events.responses),
            state: &initialized_state(),
            params: &mut params,
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        manager.send_comm_responses(&mut board, &mut comm_events);
        assert_eq!(manager.comm_link().cmd_ack_count, 1);

        let ack = manager.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn board_command_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::WriteParams,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());

        let request = command_events.board_command_requests.pop().unwrap();
        assert!(matches!(request.command, RosflightCmd::WriteParams));
    }

    #[test]
    fn rc_trim_calibration_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::RcCalibration,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());

        let request = command_events.rc_trim_calibration_requests.pop().unwrap();
        assert!(matches!(request.command, RosflightCmd::RcCalibration));
    }

    #[test]
    fn reset_origin_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::ResetOrigin,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());

        let request = command_events.reset_origin_requests.pop().unwrap();
        assert!(matches!(request.command, RosflightCmd::ResetOrigin));
    }

    #[test]
    fn send_all_config_infos_emits_request_and_defers_ack() {
        let mut board = TestBoard::default();
        let mut manager = CommManager::new(RecordingCommLink::new(), board.clock_micros());
        let mut param_events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut command_events = CommandEventQueues::default();

        manager.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SendAllConfigInfos,
        });

        manager.act_on_messages(
            &mut param_events,
            &mut comm_events,
            &mut command_events,
            &mut companion_events(),
            &mut board,
        );

        assert_eq!(manager.comm_link().cmd_ack_count, 0);
        assert!(comm_events.responses.is_empty());

        let request = command_events.config_info_requests.pop().unwrap();
        assert!(matches!(request.command, RosflightCmd::SendAllConfigInfos));
    }
}
