use crate::comm::messages::{
    enums::{OffboardControlIgnore, OffboardControlMode},
    messages::OffboardControlMsg,
};
use crate::params::{ParamId, ParamValue, Params};
use crate::rc::{Rc, Stick, Switch};
use crate::state_machine::{ErrorFlag, Event, StateManager};

pub mod service;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlType {
    Rate,        // Channel is is in rate mode (rad/s)
    Angle,       // Channel command is in angle mode (rad)
    Throttle,    // Channel is controlling throttle setting
    Passthrough, // Channel directly passes PWM input to the mixer
}

// simpler than the enum representation in c++ command_manager.h
pub(crate) const ATTITUDE_RATE_MODE: i32 = 0;
#[cfg(test)]
const ATTITUDE_ANGLE_MODE: i32 = 1;

pub const OVERRIDE_NO_OVERRIDE: u16 = 0x0;
pub const OVERRIDE_ATT_SWITCH: u16 = 0x1;
pub const OVERRIDE_THR_SWITCH: u16 = 0x2;
pub const OVERRIDE_X: u16 = 0x4;
pub const OVERRIDE_Y: u16 = 0x8;
pub const OVERRIDE_Z: u16 = 0x10;
pub const OVERRIDE_T: u16 = 0x20;
pub const OVERRIDE_OFFBOARD_X_INACTIVE: u16 = 0x40;
pub const OVERRIDE_OFFBOARD_Y_INACTIVE: u16 = 0x80;
pub const OVERRIDE_OFFBOARD_Z_INACTIVE: u16 = 0x100;
pub const OVERRIDE_OFFBOARD_T_INACTIVE: u16 = 0x200;

#[derive(Clone, Copy, Debug)]
pub struct ControlChannel {
    pub active: bool,
    pub control_type: ControlType,
    pub value: f32,
}

impl Default for ControlChannel {
    fn default() -> Self {
        Self {
            active: false,
            control_type: ControlType::Rate,
            value: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CombinedControl {
    pub stamp_ms: u32,
    pub qx: ControlChannel,
    pub qy: ControlChannel,
    pub qz: ControlChannel,
    pub fx: ControlChannel,
    pub fy: ControlChannel,
    pub fz: ControlChannel,
    pub passthrough: [ControlChannel; 4],
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum RcAngleModeLockoutState {
    #[default]
    Inactive,
    SwitchResetRequired {
        estimator_recovered: bool,
        rate_wait_logged: bool,
    },
    ParamRateUntilExplicitAngle,
}

#[derive(Clone, Copy, Debug, Default)]
struct RcAngleModeLockoutMachine {
    state: RcAngleModeLockoutState,
    denial_active: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct RcAngleModeLockoutInput {
    fixed_wing: bool,
    estimator_unhealthy: bool,
    offboard_angle_requested: bool,
    rc_angle_requested: bool,
    rc_attitude_source_requested: bool,
    att_type_switch_mapped: bool,
    att_type_switch_rate: bool,
    param_mode_rate: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct RcAngleModeLockoutOutput {
    force_rc_attitude: bool,
    force_rc_rate: bool,
    force_param_rate: bool,
}

impl RcAngleModeLockoutMachine {
    fn step(&mut self, input: RcAngleModeLockoutInput) -> RcAngleModeLockoutOutput {
        if input.fixed_wing {
            self.state = RcAngleModeLockoutState::Inactive;
            self.denial_active = false;
            return RcAngleModeLockoutOutput::default();
        }

        self.advance(input);

        let lockout_active = !matches!(self.state, RcAngleModeLockoutState::Inactive);
        let unsafe_angle_requested = input.estimator_unhealthy
            && (input.offboard_angle_requested
                || (input.rc_attitude_source_requested && input.rc_angle_requested));

        if unsafe_angle_requested {
            self.enter(input);
        } else {
            self.denial_active = false;
        }

        RcAngleModeLockoutOutput {
            force_rc_attitude: input.estimator_unhealthy && input.offboard_angle_requested,
            force_rc_rate: lockout_active || unsafe_angle_requested,
            force_param_rate: !input.att_type_switch_mapped
                && unsafe_angle_requested
                && !input.param_mode_rate,
        }
    }

    fn advance(&mut self, input: RcAngleModeLockoutInput) {
        match self.state {
            RcAngleModeLockoutState::Inactive => {}
            RcAngleModeLockoutState::SwitchResetRequired {
                estimator_recovered,
                rate_wait_logged,
            } => {
                let recovered = estimator_recovered || !input.estimator_unhealthy;
                if recovered && input.att_type_switch_rate {
                    self.state = RcAngleModeLockoutState::Inactive;
                    if !estimator_recovered {
                        crate::log_info!(
                            "Firmware Lockout: Estimator healthy: angle mode can be re-enabled"
                        );
                    }
                } else {
                    if !estimator_recovered && recovered {
                        crate::log_info!(
                            "Firmware Lockout: Estimator healthy: angle mode can be re-enabled"
                        );
                    }
                    if input.att_type_switch_rate && !recovered && !rate_wait_logged {
                        crate::log_info!(
                            "Firmware Lockout: Angle mode switch will be available once estimator reacquires"
                        );
                    }
                    self.state = RcAngleModeLockoutState::SwitchResetRequired {
                        estimator_recovered: recovered,
                        rate_wait_logged: rate_wait_logged
                            || (input.att_type_switch_rate && !recovered),
                    };
                }
            }
            RcAngleModeLockoutState::ParamRateUntilExplicitAngle => {
                if !input.estimator_unhealthy && input.param_mode_rate {
                    self.state = RcAngleModeLockoutState::Inactive;
                    crate::log_info!(
                        "Firmware Lockout: Estimator healthy: angle mode can be re-enabled"
                    );
                }
            }
        }
    }

    fn enter(&mut self, input: RcAngleModeLockoutInput) {
        if self.denial_active {
            return;
        }
        self.denial_active = true;

        crate::log_error!("Firmware Lockout: Unhealthy estimator: forced RC rate mode");
        if input.att_type_switch_mapped {
            if !matches!(
                self.state,
                RcAngleModeLockoutState::SwitchResetRequired { .. }
            ) {
                self.state = RcAngleModeLockoutState::SwitchResetRequired {
                    estimator_recovered: false,
                    rate_wait_logged: false,
                };
            }
            crate::log_error!(
                "Firmware Lockout: Move RC attitude switch to rate before angle mode can be re-enabled"
            );
        } else {
            self.state = RcAngleModeLockoutState::ParamRateUntilExplicitAngle;
            if !input.param_mode_rate {
                crate::log_error!(
                    "Firmware Lockout: RC_ATT_MODE set to rate; set RC_ATT_MODE to angle after estimator recovery to re-enable angle mode"
                );
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CommandRunResult {
    pub force_rc_attitude_mode_rate: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct EstimatorModeSafety {
    force_rc_attitude: bool,
    force_rc_rate: bool,
    force_param_rate: bool,
}

#[derive(Default)]
pub struct CommandManager {
    // Command structs
    rc_command: CombinedControl,
    offboard_command: CombinedControl,
    combined_command: CombinedControl,
    multirotor_failsafe_command: CombinedControl,
    fixedwing_failsafe_command: CombinedControl,
    last_offboard_command_us: u64,
    last_stick_override_time: [u32; 3], // for x, y, and z sticks

    // State flags
    rc_throttle_override: bool,
    rc_attitude_override: bool,
    rc_override: u16,
    rc_angle_mode_lockout: RcAngleModeLockoutMachine,
}

impl CommandManager {
    /// Creates a new CommandManager with default initial values.
    pub fn new() -> Self {
        Self {
            multirotor_failsafe_command: CombinedControl {
                qx: ControlChannel {
                    active: true,
                    control_type: ControlType::Angle,
                    value: 0.0,
                },
                qy: ControlChannel {
                    active: true,
                    control_type: ControlType::Angle,
                    value: 0.0,
                },
                qz: ControlChannel {
                    active: true,
                    control_type: ControlType::Rate,
                    value: 0.0,
                },
                fx: ControlChannel {
                    active: true,
                    control_type: ControlType::Throttle,
                    value: 0.0,
                },
                fy: ControlChannel {
                    active: true,
                    control_type: ControlType::Throttle,
                    value: 0.0,
                },
                fz: ControlChannel {
                    active: true,
                    control_type: ControlType::Throttle,
                    value: 0.0,
                },
                ..Default::default()
            },
            fixedwing_failsafe_command: CombinedControl {
                qx: ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.0,
                },
                qy: ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.0,
                },
                qz: ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.0,
                },
                fx: ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.0,
                },
                fy: ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.0,
                },
                fz: ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.0,
                },
                ..Default::default()
            },
            ..Default::default()
        }
    }

    pub fn init(&mut self, params: &Params, state_manager: &mut StateManager) {
        self.update_failsafe_config(params, state_manager);
    }

    pub fn update_failsafe_config(&mut self, params: &Params, state_manager: &mut StateManager) {
        let mut failsafe_throttle = match params.get_by_id(ParamId::PARAM_FAILSAFE_THROTTLE) {
            ParamValue::Float(val) => val,
            _ => 0.0f32,
        };

        let is_fixed_wing = match params.get_by_id(ParamId::PARAM_FIXED_WING) {
            ParamValue::Int(val) => val != 0,
            _ => false,
        };

        if !is_fixed_wing && (failsafe_throttle < 0.0 || failsafe_throttle > 1.0) {
            state_manager.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_FAILSAFE), params);
            failsafe_throttle = 0.0f32;
        } else {
            state_manager.update(Event::ERROR_CLEARED(ErrorFlag::INVALID_FAILSAFE), params);
        }

        self.multirotor_failsafe_command.fx.value = 0.0;
        self.multirotor_failsafe_command.fy.value = 0.0;
        self.multirotor_failsafe_command.fz.value = 0.0;

        match params.get_by_id(ParamId::PARAM_RC_F_AXIS) {
            // The "happy path": it's an Int, as expected
            ParamValue::Int(axis) => {
                match axis {
                    // Note: `axis` is an i32, not `&i32`, so no deref `*` needed
                    0 => self.multirotor_failsafe_command.fx.value = failsafe_throttle,
                    1 => self.multirotor_failsafe_command.fy.value = failsafe_throttle,
                    _ => self.multirotor_failsafe_command.fz.value = failsafe_throttle,
                }
            }
            // Error case: it's the wrong type.
            // We log the error and apply a safe default (Fz).
            _ => {
                self.multirotor_failsafe_command.fz.value = failsafe_throttle;
            }
        }

        self.fixedwing_failsafe_command.fx.value = 0.0;
        self.fixedwing_failsafe_command.fy.value = 0.0;
        self.fixedwing_failsafe_command.fz.value = 0.0;
    }

    pub fn run(
        &mut self,
        now_ms: u32,
        params: &Params,
        rc: &mut Rc,
        state_manager: &mut StateManager, // <-- Must be mutable
    ) -> CommandRunResult {
        let now_us = now_ms as u64 * 1000;
        let mut result = CommandRunResult {
            force_rc_attitude_mode_rate: false,
        };

        if !rc.check_rc_health(now_us, params) {
            state_manager.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), params);
        }

        // --- 1. Failsafe Action (C++ lines 240-243) ---
        // This is the highest priority. If in failsafe, override all commands.
        if state_manager.is_in_failsafe() {
            let is_fixed_wing = match params.get_by_id(ParamId::PARAM_FIXED_WING) {
                ParamValue::Int(val) => val != 0,
                _ => false,
            };
            self.combined_command = if is_fixed_wing {
                self.fixedwing_failsafe_command
            } else {
                self.multirotor_failsafe_command
            };
            return result; // Exit early
        }

        // --- 3. Offboard Timeout "Fail-over" (C++ lines 246-252) ---
        self.update_offboard_timeout(now_us, params);

        let safety = self.estimator_mode_safety(params, rc, state_manager);
        result.force_rc_attitude_mode_rate = safety.force_param_rate;

        // --- 2. RC Update (C++ line 244) ---
        // Fresh RC packets update the RC input command. Combined command resolution runs below
        // even without fresh RC so offboard updates and timeouts are not gated by RC cadence.
        let has_new_rc = rc.new_command();
        if has_new_rc || safety.force_rc_rate {
            self.interpret_rc(
                rc,
                params,
                safety.force_rc_rate.then_some(ControlType::Rate),
            );
        }

        // --- 4. Muxing (C++ lines 253-256) ---
        self.do_muxing(params, rc, now_ms, safety.force_rc_attitude);

        result
    }

    fn update_offboard_timeout(&mut self, now_us: u64, params: &Params) {
        let timeout_ms = match params.get_by_id(ParamId::PARAM_OFFBOARD_TIMEOUT) {
            ParamValue::Int(val) => val as u32,
            _ => 100,
        };

        if now_us <= self.last_offboard_command_us + (timeout_ms as u64 * 1000) {
            return;
        }

        self.offboard_command.qx.active = false;
        self.offboard_command.qy.active = false;
        self.offboard_command.qz.active = false;
        self.offboard_command.fx.active = false;
        self.offboard_command.fy.active = false;
        self.offboard_command.fz.active = false;
        for channel in &mut self.offboard_command.passthrough {
            channel.active = false;
        }
    }

    /// Receives a new offboard control command.
    /// This should be called from CommManager::act_on_messages
    pub fn set_new_offboard_command(
        &mut self,
        now_us: u64,
        msg: &OffboardControlMsg,
        _params: &Params,
    ) {
        // We got a new command, so update the timestamp
        self.last_offboard_command_us = now_us;
        self.offboard_command.stamp_ms = (now_us / 1000) as u32;

        self.offboard_command.qx.value = msg.qx;
        self.offboard_command.qy.value = msg.qy;
        self.offboard_command.qz.value = msg.qz;
        self.offboard_command.fx.value = msg.fx;
        self.offboard_command.fy.value = msg.fy;
        self.offboard_command.fz.value = msg.fz;
        for (channel, value) in self
            .offboard_command
            .passthrough
            .iter_mut()
            .zip(msg.passthrough.iter())
        {
            channel.value = *value;
            channel.control_type = ControlType::Passthrough;
        }

        self.offboard_command.qx.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_QX);
        self.offboard_command.qy.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_QY);
        self.offboard_command.qz.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_QZ);
        self.offboard_command.fx.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_FX);
        self.offboard_command.fy.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_FY);
        self.offboard_command.fz.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_FZ);
        self.offboard_command.passthrough[0].active =
            !msg.ignore.contains(OffboardControlIgnore::IGNORE_PASS_0);
        self.offboard_command.passthrough[1].active =
            !msg.ignore.contains(OffboardControlIgnore::IGNORE_PASS_1);
        self.offboard_command.passthrough[2].active =
            !msg.ignore.contains(OffboardControlIgnore::IGNORE_PASS_2);
        self.offboard_command.passthrough[3].active =
            !msg.ignore.contains(OffboardControlIgnore::IGNORE_PASS_3);

        match msg.mode {
            OffboardControlMode::ModePassThrough => {
                self.offboard_command.qx.control_type = ControlType::Passthrough;
                self.offboard_command.qy.control_type = ControlType::Passthrough;
                self.offboard_command.qz.control_type = ControlType::Passthrough;
                self.offboard_command.fx.control_type = ControlType::Passthrough;
                self.offboard_command.fy.control_type = ControlType::Passthrough;
                self.offboard_command.fz.control_type = ControlType::Passthrough;
            }
            OffboardControlMode::ModeRollratePitchrateYawrateThrottle => {
                self.offboard_command.qx.control_type = ControlType::Rate;
                self.offboard_command.qy.control_type = ControlType::Rate;
                self.offboard_command.qz.control_type = ControlType::Rate;
                self.offboard_command.fx.control_type = ControlType::Throttle;
                self.offboard_command.fy.control_type = ControlType::Throttle;
                self.offboard_command.fz.control_type = ControlType::Throttle;
            }
            OffboardControlMode::ModeRollPitchYawrateThrottle => {
                self.offboard_command.qx.control_type = ControlType::Angle;
                self.offboard_command.qy.control_type = ControlType::Angle;
                self.offboard_command.qz.control_type = ControlType::Rate;
                self.offboard_command.fx.control_type = ControlType::Throttle;
                self.offboard_command.fy.control_type = ControlType::Throttle;
                self.offboard_command.fz.control_type = ControlType::Throttle;
            }
        }
    }

    /// Port of C++ `interpret_rc` (command_manager.cpp:101)
    fn interpret_rc(
        &mut self,
        rc: &Rc,
        params: &Params,
        forced_roll_pitch_type: Option<ControlType>,
    ) {
        // Read all stick values from the RC unit
        self.rc_command.qx.value = rc.stick(Stick::X);
        self.rc_command.qy.value = rc.stick(Stick::Y);
        self.rc_command.qz.value = rc.stick(Stick::Z);
        let f_stick_value = rc.stick(Stick::F);

        // C++: line 109, logic for F-axis (type-safe)
        match params.get_by_id(ParamId::PARAM_RC_F_AXIS) {
            // "Happy path": it's an Int as expected
            ParamValue::Int(axis) => {
                match axis {
                    0 => {
                        // X_AXIS
                        self.rc_command.fx.value = f_stick_value;
                        self.rc_command.fy.value = 0.0;
                        self.rc_command.fz.value = 0.0;
                    }
                    1 => {
                        // Y_AXIS
                        self.rc_command.fx.value = 0.0;
                        self.rc_command.fy.value = f_stick_value;
                        self.rc_command.fz.value = 0.0;
                    }
                    _ => {
                        // Z_AXIS (default)
                        self.rc_command.fx.value = 0.0;
                        self.rc_command.fy.value = 0.0;
                        self.rc_command.fz.value = f_stick_value;
                    }
                }
            }
            // Error case: param is wrong type!
            _ => {
                // Default to Z_AXIS for safety
                self.rc_command.fx.value = 0.0;
                self.rc_command.fy.value = 0.0;
                self.rc_command.fz.value = f_stick_value;
            }
        }

        // All RC channels are always active... active for rc doesn't really mean anything. Active on offboard command means override rc...
        self.rc_command.qx.active = true;
        self.rc_command.qy.active = true;
        self.rc_command.qz.active = true;
        self.rc_command.fx.active = true;
        self.rc_command.fy.active = true;
        self.rc_command.fz.active = true;

        // C++: lines 130-153
        let is_fixed_wing = match params.get_by_id(ParamId::PARAM_FIXED_WING) {
            ParamValue::Int(val) => val != 0,
            _ => {
                // This is a param definition error. Default to the safer
                // (non-fixed-wing) case.
                false
            }
        };
        if is_fixed_wing {
            self.rc_command.qx.control_type = ControlType::Passthrough;
            self.rc_command.qy.control_type = ControlType::Passthrough;
            self.rc_command.qz.control_type = ControlType::Passthrough;
            self.rc_command.fx.control_type = ControlType::Passthrough;
            self.rc_command.fy.control_type = ControlType::Passthrough;
            self.rc_command.fz.control_type = ControlType::Passthrough;
        } else {
            // check if we've mapped the AttType channel...
            let roll_pitch_type = if let Some(forced_roll_pitch_type) = forced_roll_pitch_type {
                forced_roll_pitch_type
            } else if rc.switch_mapped(Switch::AttType) {
                // if we have, probe it to know if we should use rate or angle for qx and qy
                if rc.switch_on(Switch::AttType) {
                    ControlType::Angle
                } else {
                    ControlType::Rate
                }
            } else {
                // if not, fall back to the parameter Attitude mode
                let att_mode = match params.get_by_id(ParamId::PARAM_RC_ATTITUDE_MODE) {
                    ParamValue::Int(val) => val,
                    _ => {
                        200 // Default value from C++ params
                    }
                };
                match att_mode {
                    ATTITUDE_RATE_MODE => ControlType::Rate,
                    _ => {
                        // if we're not in rate mode, we're in pitch mode...
                        ControlType::Angle
                    }
                }
            };

            self.rc_command.qx.control_type = roll_pitch_type;
            self.rc_command.qy.control_type = roll_pitch_type;

            match roll_pitch_type {
                ControlType::Rate => {
                    let max_rollrate = match params.get_by_id(ParamId::PARAM_RC_MAX_ROLLRATE) {
                        ParamValue::Float(val) => val,
                        _ => 1.0,
                    };
                    let max_pitchrate = match params.get_by_id(ParamId::PARAM_RC_MAX_PITCHRATE) {
                        ParamValue::Float(val) => val,
                        _ => 1.0,
                    };
                    self.rc_command.qx.value *= max_rollrate;
                    self.rc_command.qy.value *= max_pitchrate;
                }
                ControlType::Angle => {
                    let max_roll = match params.get_by_id(ParamId::PARAM_RC_MAX_ROLL) {
                        ParamValue::Float(val) => val,
                        _ => 1.0,
                    };
                    let max_pitch = match params.get_by_id(ParamId::PARAM_RC_MAX_PITCH) {
                        ParamValue::Float(val) => val,
                        _ => 1.0,
                    };
                    self.rc_command.qx.value *= max_roll;
                    self.rc_command.qy.value *= max_pitch;
                }
                _ => {}
            }

            self.rc_command.qz.control_type = ControlType::Rate;
            let max_yawrate = match params.get_by_id(ParamId::PARAM_RC_MAX_YAWRATE) {
                ParamValue::Float(val) => val,
                _ => 1.0,
            };
            self.rc_command.qz.value *= max_yawrate;

            self.rc_command.fx.control_type = ControlType::Throttle;
            self.rc_command.fy.control_type = ControlType::Throttle;
            self.rc_command.fz.control_type = ControlType::Throttle;
        }
    }

    /// This is the C++ `run` function's muxing logic
    fn do_muxing(&mut self, params: &Params, rc: &Rc, now_ms: u32, force_rc_attitude: bool) {
        // C++: command_manager.cpp (lines 253-256)
        let attitude_override = self.do_attitude_muxing(params, rc, now_ms, force_rc_attitude);
        let throttle_override = self.do_throttle_muxing(params, rc);
        self.rc_override = attitude_override | throttle_override;
        self.rc_attitude_override = attitude_override != OVERRIDE_NO_OVERRIDE;
        self.rc_throttle_override = throttle_override != OVERRIDE_NO_OVERRIDE;
        self.combined_command.passthrough = self.offboard_command.passthrough;
    }

    fn attitude_stick_deviated(
        &mut self,
        rc: &Rc,
        stick: Stick,
        deviation_param: f32,
        lag_time_ms: u32,
        now_ms: u32,
    ) -> bool {
        if now_ms < self.last_stick_override_time[stick as usize].saturating_add(lag_time_ms) {
            return true;
        }

        if (rc.stick(stick)).abs() <= deviation_param {
            return false;
        }

        self.last_stick_override_time[stick as usize] = now_ms;
        true
    }

    fn do_attitude_muxing(
        &mut self,
        params: &Params,
        rc: &Rc,
        now_ms: u32,
        force_rc_attitude: bool,
    ) -> u16 {
        let deviation_param = match params.get_by_id(ParamId::PARAM_RC_OVERRIDE_DEVIATION) {
            ParamValue::Float(val) => val,
            _ => {
                0.1 // Default value from C++ params
            }
        };

        // --- REFACTORED: PARAM_OVERRIDE_LAG_TIME ---
        let lag_time_ms = match params.get_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME) {
            ParamValue::Int(val) => val as u32,
            _ => {
                200 // Default value from C++ params
            }
        };

        let switch_override =
            rc.switch_mapped(Switch::AttOverride) && rc.switch_on(Switch::AttOverride);

        let mut override_mask = if switch_override || force_rc_attitude {
            OVERRIDE_ATT_SWITCH
        } else {
            OVERRIDE_NO_OVERRIDE
        };

        let x_stick_deviated =
            self.attitude_stick_deviated(rc, Stick::X, deviation_param, lag_time_ms, now_ms);
        if x_stick_deviated {
            override_mask |= OVERRIDE_X;
        }
        if !self.offboard_command.qx.active {
            override_mask |= OVERRIDE_OFFBOARD_X_INACTIVE;
        }
        self.combined_command.qx = if force_rc_attitude
            || switch_override
            || x_stick_deviated
            || !self.offboard_command.qx.active
        {
            self.rc_command.qx
        } else {
            self.offboard_command.qx
        };

        let y_stick_deviated =
            self.attitude_stick_deviated(rc, Stick::Y, deviation_param, lag_time_ms, now_ms);
        if y_stick_deviated {
            override_mask |= OVERRIDE_Y;
        }
        if !self.offboard_command.qy.active {
            override_mask |= OVERRIDE_OFFBOARD_Y_INACTIVE;
        }
        self.combined_command.qy = if force_rc_attitude
            || switch_override
            || y_stick_deviated
            || !self.offboard_command.qy.active
        {
            self.rc_command.qy
        } else {
            self.offboard_command.qy
        };

        let z_stick_deviated =
            self.attitude_stick_deviated(rc, Stick::Z, deviation_param, lag_time_ms, now_ms);
        if z_stick_deviated {
            override_mask |= OVERRIDE_Z;
        }
        if !self.offboard_command.qz.active {
            override_mask |= OVERRIDE_OFFBOARD_Z_INACTIVE;
        }
        self.combined_command.qz =
            if switch_override || z_stick_deviated || !self.offboard_command.qz.active {
                self.rc_command.qz
            } else {
                self.offboard_command.qz
            };

        override_mask
    }

    fn estimator_mode_safety(
        &mut self,
        params: &Params,
        rc: &Rc,
        state_manager: &StateManager,
    ) -> EstimatorModeSafety {
        let estimator_unhealthy = state_manager
            .get_errors()
            .contains(ErrorFlag::UNHEALTHY_ESTIMATOR);
        let fixed_wing = matches!(
            params.get_by_id(ParamId::PARAM_FIXED_WING),
            ParamValue::Int(value) if value != 0
        );

        let lockout = self.rc_angle_mode_lockout.step(RcAngleModeLockoutInput {
            fixed_wing,
            estimator_unhealthy,
            offboard_angle_requested: self.offboard_angle_requested(),
            rc_angle_requested: rc_angle_mode_selected(params, rc),
            rc_attitude_source_requested: self.rc_attitude_source_requested(rc),
            att_type_switch_mapped: rc.switch_mapped(Switch::AttType),
            att_type_switch_rate: !rc.switch_on(Switch::AttType),
            param_mode_rate: param_rc_attitude_mode_is_rate(params),
        });

        EstimatorModeSafety {
            force_rc_attitude: lockout.force_rc_attitude,
            force_rc_rate: lockout.force_rc_rate,
            force_param_rate: lockout.force_param_rate,
        }
    }

    fn offboard_angle_requested(&self) -> bool {
        (self.offboard_command.qx.active
            && self.offboard_command.qx.control_type == ControlType::Angle)
            || (self.offboard_command.qy.active
                && self.offboard_command.qy.control_type == ControlType::Angle)
    }

    fn rc_attitude_source_requested(&self, rc: &Rc) -> bool {
        let switch_override =
            rc.switch_mapped(Switch::AttOverride) && rc.switch_on(Switch::AttOverride);
        switch_override || !self.offboard_command.qx.active || !self.offboard_command.qy.active
    }

    fn do_throttle_muxing(&mut self, params: &Params, rc: &Rc) -> u16 {
        let throttle_axis_idx = match params.get_by_id(ParamId::PARAM_RC_F_AXIS) {
            ParamValue::Int(val) => val as u32,
            _ => {
                200 // Default value from C++ params
            }
        };

        let (rc_throttle_value, offboard_throttle_channel) = match throttle_axis_idx {
            0 => (self.rc_command.fx.value, &self.offboard_command.fx),
            1 => (self.rc_command.fy.value, &self.offboard_command.fy),
            _ => (self.rc_command.fz.value, &self.offboard_command.fz),
        };

        let mut override_mask = OVERRIDE_NO_OVERRIDE;

        if rc.switch_mapped(Switch::ThrottleOverride) && rc.switch_on(Switch::ThrottleOverride) {
            override_mask |= OVERRIDE_THR_SWITCH;
        }

        if offboard_throttle_channel.active {
            let take_min = match params.get_by_id(ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE) {
                ParamValue::Int(val) => val != 0,
                _ => true,
            };

            if take_min && rc_throttle_value < offboard_throttle_channel.value {
                override_mask |= OVERRIDE_T;
            }
        } else {
            override_mask |= OVERRIDE_OFFBOARD_T_INACTIVE;
        }

        if override_mask != OVERRIDE_NO_OVERRIDE {
            self.combined_command.fx = self.rc_command.fx;
            self.combined_command.fy = self.rc_command.fy;
            self.combined_command.fz = self.rc_command.fz;
        } else {
            self.combined_command.fx = self.offboard_command.fx;
            self.combined_command.fy = self.offboard_command.fy;
            self.combined_command.fz = self.offboard_command.fz;
        }

        override_mask
    }

    pub fn combined_control(&self) -> &CombinedControl {
        &self.combined_command
    }

    pub fn rc_control(&self) -> &CombinedControl {
        &self.rc_command
    }

    pub fn get_control_mode(&self) -> ControlType {
        // We assume the qx (roll) channel represents the
        // primary attitude control mode.
        self.combined_command.qx.control_type
    }

    pub fn rc_override_active(&self) -> bool {
        self.rc_override != OVERRIDE_NO_OVERRIDE
    }

    pub fn get_rc_override(&self) -> u16 {
        self.rc_override
    }

    /// Port of C++ `offboard_control_active` (line 220)
    pub fn is_offboard_active(&self) -> bool {
        // This is true if *any* offboard channel is active
        self.offboard_command.qx.active
            || self.offboard_command.qy.active
            || self.offboard_command.qz.active
            || self.offboard_command.fx.active
            || self.offboard_command.fy.active
            || self.offboard_command.fz.active
    }
}

fn rc_angle_mode_selected(params: &Params, rc: &Rc) -> bool {
    if rc.switch_mapped(Switch::AttType) {
        rc.switch_on(Switch::AttType)
    } else {
        !param_rc_attitude_mode_is_rate(params)
    }
}

fn param_rc_attitude_mode_is_rate(params: &Params) -> bool {
    matches!(
        params.get_by_id(ParamId::PARAM_RC_ATTITUDE_MODE),
        ParamValue::Int(ATTITUDE_RATE_MODE)
    )
}

impl From<ControlType> for OffboardControlMode {
    fn from(val: ControlType) -> Self {
        match val {
            ControlType::Rate => OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
            ControlType::Passthrough => OffboardControlMode::ModePassThrough,
            ControlType::Angle => OffboardControlMode::ModeRollPitchYawrateThrottle,
            ControlType::Throttle => OffboardControlMode::ModeRollPitchYawrateThrottle,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::log::Logger;
    use crate::packets::{RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader};

    fn initialized_rc(params: &Params) -> Rc {
        let mut rc = Rc::new();
        rc.init(params);
        rc
    }

    fn receive_rc(
        rc: &mut Rc,
        params: &Params,
        state: &mut StateManager,
        channels: [f32; RC_PACKET_CHANNELS],
    ) {
        rc.receive(&RcPacket {
            header: RosflightPacketHeader {
                timestamp: 100_000,
                status: 0,
            },
            n_chan: 8,
            chan: channels,
            lol: false,
        });
        rc.run(100, params, state);
    }

    fn clear_logs() {
        while Logger::pop().is_some() {}
    }

    fn drain_log_count() -> usize {
        let mut count = 0;
        while Logger::pop().is_some() {
            count += 1;
        }
        count
    }

    fn set_unhealthy_estimator(state: &mut StateManager, params: &Params, unhealthy: bool) {
        state.set_error_flag(ErrorFlag::UNHEALTHY_ESTIMATOR, unhealthy, params);
    }

    fn offboard_msg(mode: OffboardControlMode) -> OffboardControlMsg {
        OffboardControlMsg {
            mode,
            ignore: OffboardControlIgnore::empty(),
            qx: 0.25,
            qy: -0.5,
            qz: 0.75,
            fx: 0.0,
            fy: 0.0,
            fz: 0.4,
            passthrough: [0.0; 4],
        }
    }

    #[test]
    fn rc_override_status_reports_upstream_stick_and_throttle_bits() {
        let params = Params::new();
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[0] = 0.7;
        channels[2] = 0.2;

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.8,
                fy: 0.8,
                fz: 0.8,
                passthrough: [0.0; 4],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, channels);

        command.run(1000, &params, &mut rc, &mut state);

        assert_eq!(command.get_rc_override(), OVERRIDE_X | OVERRIDE_T);
        assert!(command.rc_override_active());
    }

    #[test]
    fn take_min_throttle_uses_rc_when_rc_throttle_is_lower() {
        let mut params = Params::new();
        params.set_by_id(
            ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,
            ParamValue::Int(1),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[2] = 0.2;

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: 0.8,
                passthrough: [0.0; 4],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, channels);

        command.run(1000, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(command.get_rc_override(), OVERRIDE_T);
        assert_eq!(combined.fz.control_type, ControlType::Throttle);
        assert!((combined.fz.value - 0.2).abs() < 1e-6);
    }

    #[test]
    fn take_min_throttle_keeps_offboard_when_offboard_throttle_is_lower() {
        let mut params = Params::new();
        params.set_by_id(
            ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,
            ParamValue::Int(1),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[2] = 0.8;

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: 0.2,
                passthrough: [0.0; 4],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, channels);

        command.run(1000, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(command.get_rc_override(), OVERRIDE_NO_OVERRIDE);
        assert_eq!(combined.fz.control_type, ControlType::Throttle);
        assert!((combined.fz.value - 0.2).abs() < 1e-6);
    }

    #[test]
    fn take_min_throttle_uses_rc_when_throttle_switch_on_and_rc_throttle_is_lower() {
        let mut params = Params::new();
        params.set_by_id(
            ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,
            ParamValue::Int(1),
        );
        params.set_by_id(
            ParamId::PARAM_RC_THROTTLE_OVERRIDE_CHANNEL,
            ParamValue::Int(5),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[2] = 0.2;
        channels[5] = 1.0;

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: 0.8,
                passthrough: [0.0; 4],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, channels);

        command.run(1000, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(command.get_rc_override(), OVERRIDE_THR_SWITCH | OVERRIDE_T);
        assert_eq!(combined.fz.control_type, ControlType::Throttle);
        assert!((combined.fz.value - 0.2).abs() < 1e-6);
    }

    #[test]
    fn take_min_throttle_with_passthrough_ned_thrust_documents_current_muxing() {
        let mut params = Params::new();
        params.set_by_id(
            ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,
            ParamValue::Int(1),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[2] = 0.2;

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModePassThrough,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: -25.0,
                passthrough: [0.0; 4],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, channels);

        command.run(1000, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(command.get_rc_override(), OVERRIDE_NO_OVERRIDE);
        assert_eq!(combined.fz.control_type, ControlType::Passthrough);
        assert_eq!(combined.fz.value, -25.0);
    }

    #[test]
    fn rc_override_status_reports_inactive_offboard_channel_bits() {
        let params = Params::new();
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::IGNORE_QY | OffboardControlIgnore::IGNORE_FZ,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: 0.8,
                passthrough: [0.0; 4],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);

        command.run(1000, &params, &mut rc, &mut state);

        assert_eq!(
            command.get_rc_override(),
            OVERRIDE_OFFBOARD_Y_INACTIVE | OVERRIDE_OFFBOARD_T_INACTIVE
        );
        assert!(command.rc_override_active());
    }

    #[test]
    fn offboard_update_resolves_combined_command_without_new_rc_packet() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);

        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);
        command.run(100, &params, &mut rc, &mut state);

        command.set_new_offboard_command(
            110_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.25,
                qy: -0.5,
                qz: 0.75,
                fx: 0.0,
                fy: 0.0,
                fz: 0.4,
                passthrough: [0.0; 4],
            },
            &params,
        );

        command.run(110, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(combined.qx.control_type, ControlType::Rate);
        assert_eq!(combined.qx.value, 0.25);
        assert_eq!(combined.qy.value, -0.5);
        assert_eq!(combined.qz.value, 0.75);
        assert_eq!(combined.fz.value, 0.4);
        assert_eq!(command.get_rc_override(), OVERRIDE_NO_OVERRIDE);
    }

    #[test]
    fn healthy_estimator_allows_offboard_angle_mode() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);

        command.set_new_offboard_command(
            1_000_000,
            &offboard_msg(OffboardControlMode::ModeRollPitchYawrateThrottle),
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);

        let result = command.run(1000, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(combined.qx.control_type, ControlType::Angle);
        assert_eq!(combined.qx.value, 0.25);
        assert_eq!(combined.qy.control_type, ControlType::Angle);
        assert_eq!(combined.qy.value, -0.5);
        assert_eq!(command.get_rc_override(), OVERRIDE_NO_OVERRIDE);
        assert!(!result.force_rc_attitude_mode_rate);
    }

    #[test]
    fn unhealthy_estimator_allows_offboard_rate_and_passthrough_modes() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        set_unhealthy_estimator(&mut state, &params, true);

        command.set_new_offboard_command(
            1_000_000,
            &offboard_msg(OffboardControlMode::ModeRollratePitchrateYawrateThrottle),
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);
        let result = command.run(1000, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );
        assert_eq!(command.combined_control().qx.value, 0.25);
        assert!(!result.force_rc_attitude_mode_rate);

        command.set_new_offboard_command(
            1_001_000,
            &offboard_msg(OffboardControlMode::ModePassThrough),
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);
        let result = command.run(1001, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Passthrough
        );
        assert_eq!(command.combined_control().qx.value, 0.25);
        assert!(!result.force_rc_attitude_mode_rate);
    }

    #[test]
    fn unhealthy_estimator_denies_offboard_angle_and_uses_rc_rate() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_RC_MAX_ROLLRATE, ParamValue::Float(2.0));
        params.set_by_id(ParamId::PARAM_RC_MAX_PITCHRATE, ParamValue::Float(3.0));
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[0] = 0.75;
        channels[1] = 0.25;
        set_unhealthy_estimator(&mut state, &params, true);

        command.set_new_offboard_command(
            1_000_000,
            &offboard_msg(OffboardControlMode::ModeRollPitchYawrateThrottle),
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, channels);

        let result = command.run(1000, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(combined.qx.control_type, ControlType::Rate);
        assert!((combined.qx.value - 1.0).abs() < 1e-6);
        assert_eq!(combined.qy.control_type, ControlType::Rate);
        assert!((combined.qy.value + 1.5).abs() < 1e-6);
        assert_eq!(
            command.get_rc_override(),
            OVERRIDE_ATT_SWITCH | OVERRIDE_X | OVERRIDE_Y
        );
        assert!(result.force_rc_attitude_mode_rate);
    }

    #[test]
    fn unhealthy_estimator_denies_rc_angle_switch_and_logs_without_spam() {
        clear_logs();
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        params.set_by_id(
            ParamId::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,
            ParamValue::Int(5),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[5] = 1.0;
        set_unhealthy_estimator(&mut state, &params, true);

        receive_rc(&mut rc, &params, &mut state, channels);
        let first = command.run(1000, &params, &mut rc, &mut state);
        receive_rc(&mut rc, &params, &mut state, channels);
        let second = command.run(1001, &params, &mut rc, &mut state);

        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );
        assert_eq!(
            command.get_rc_override(),
            OVERRIDE_OFFBOARD_X_INACTIVE
                | OVERRIDE_OFFBOARD_Y_INACTIVE
                | OVERRIDE_OFFBOARD_Z_INACTIVE
                | OVERRIDE_OFFBOARD_T_INACTIVE
        );
        assert!(!first.force_rc_attitude_mode_rate);
        assert!(!second.force_rc_attitude_mode_rate);
        assert_eq!(drain_log_count(), 2);
    }

    #[test]
    fn switch_reset_before_estimator_recovery_waits_then_unlocks_on_recovery() {
        clear_logs();
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        params.set_by_id(
            ParamId::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,
            ParamValue::Int(5),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[5] = 1.0;
        set_unhealthy_estimator(&mut state, &params, true);

        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1000, &params, &mut rc, &mut state);
        assert_eq!(drain_log_count(), 2);

        channels[5] = 0.0;
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1001, &params, &mut rc, &mut state);
        assert_eq!(drain_log_count(), 1);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );

        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1002, &params, &mut rc, &mut state);
        assert_eq!(drain_log_count(), 0);

        set_unhealthy_estimator(&mut state, &params, false);
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1003, &params, &mut rc, &mut state);
        assert_eq!(drain_log_count(), 1);

        channels[5] = 1.0;
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1004, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Angle
        );
    }

    #[test]
    fn estimator_recovery_before_switch_reset_unlocks_when_switch_moves_to_rate() {
        clear_logs();
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        params.set_by_id(
            ParamId::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,
            ParamValue::Int(5),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[5] = 1.0;
        set_unhealthy_estimator(&mut state, &params, true);

        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1000, &params, &mut rc, &mut state);
        assert_eq!(drain_log_count(), 2);

        set_unhealthy_estimator(&mut state, &params, false);
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1001, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );
        assert_eq!(drain_log_count(), 1);

        channels[5] = 0.0;
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1002, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );
        assert_eq!(drain_log_count(), 0);

        channels[5] = 1.0;
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1003, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Angle
        );
    }

    #[test]
    fn switch_must_be_rate_at_unlock_even_if_rate_was_observed_before_recovery() {
        clear_logs();
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        params.set_by_id(
            ParamId::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,
            ParamValue::Int(5),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[5] = 1.0;
        set_unhealthy_estimator(&mut state, &params, true);

        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1000, &params, &mut rc, &mut state);
        assert_eq!(drain_log_count(), 2);

        channels[5] = 0.0;
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1001, &params, &mut rc, &mut state);
        assert_eq!(drain_log_count(), 1);

        channels[5] = 1.0;
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1002, &params, &mut rc, &mut state);
        assert_eq!(drain_log_count(), 2);

        set_unhealthy_estimator(&mut state, &params, false);
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1003, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );
        assert_eq!(drain_log_count(), 1);

        channels[5] = 0.0;
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1004, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );

        channels[5] = 1.0;
        receive_rc(&mut rc, &params, &mut state, channels);
        command.run(1005, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Angle
        );
    }

    #[test]
    fn no_switch_angle_mode_requests_param_rate_until_explicit_angle_after_recovery() {
        clear_logs();
        let mut params = Params::new();
        params.set_by_id(
            ParamId::PARAM_RC_ATTITUDE_MODE,
            ParamValue::Int(ATTITUDE_ANGLE_MODE),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        set_unhealthy_estimator(&mut state, &params, true);

        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);
        let result = command.run(1000, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );
        assert!(result.force_rc_attitude_mode_rate);
        assert_eq!(drain_log_count(), 2);

        params.set_by_id(
            ParamId::PARAM_RC_ATTITUDE_MODE,
            ParamValue::Int(ATTITUDE_RATE_MODE),
        );
        set_unhealthy_estimator(&mut state, &params, false);
        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);
        command.run(1001, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Rate
        );
        assert_eq!(drain_log_count(), 1);

        params.set_by_id(
            ParamId::PARAM_RC_ATTITUDE_MODE,
            ParamValue::Int(ATTITUDE_ANGLE_MODE),
        );
        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);
        command.run(1002, &params, &mut rc, &mut state);
        assert_eq!(
            command.combined_control().qx.control_type,
            ControlType::Angle
        );
    }

    #[test]
    fn offboard_timeout_resolves_back_to_rc_without_new_rc_packet() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_OFFBOARD_TIMEOUT, ParamValue::Int(100));
        params.set_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME, ParamValue::Int(0));
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);

        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);
        command.run(100, &params, &mut rc, &mut state);
        let rc_qx = command.rc_control().qx.value;
        let rc_fz = command.rc_control().fz.value;

        command.set_new_offboard_command(
            110_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::empty(),
                qx: -0.25,
                qy: 0.5,
                qz: -0.75,
                fx: 0.0,
                fy: 0.0,
                fz: 0.4,
                passthrough: [0.0; 4],
            },
            &params,
        );
        command.run(110, &params, &mut rc, &mut state);
        assert_eq!(command.combined_control().qx.value, -0.25);

        command.run(211, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(combined.qx.value, rc_qx);
        assert_eq!(combined.fz.value, rc_fz);
        assert_eq!(
            command.get_rc_override(),
            OVERRIDE_OFFBOARD_X_INACTIVE
                | OVERRIDE_OFFBOARD_Y_INACTIVE
                | OVERRIDE_OFFBOARD_Z_INACTIVE
                | OVERRIDE_OFFBOARD_T_INACTIVE
        );
    }

    #[test]
    fn throttle_switch_override_still_reports_inactive_offboard_throttle() {
        let mut params = Params::new();
        params.set_by_id(
            ParamId::PARAM_RC_THROTTLE_OVERRIDE_CHANNEL,
            ParamValue::Int(5),
        );
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[5] = 1.0;

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::IGNORE_FZ,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: 0.8,
                passthrough: [0.0; 4],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, channels);

        command.run(1000, &params, &mut rc, &mut state);

        assert_eq!(
            command.get_rc_override(),
            OVERRIDE_THR_SWITCH | OVERRIDE_OFFBOARD_T_INACTIVE
        );
        assert!(command.rc_override_active());
    }

    #[test]
    fn rc_roll_and_pitch_scale_on_their_own_axes() {
        let params = Params::new();
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);
        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[0] = 0.75;
        channels[1] = 0.25;

        receive_rc(&mut rc, &params, &mut state, channels);
        command.interpret_rc(&rc, &params, None);

        assert_eq!(command.rc_control().qx.control_type, ControlType::Angle);
        assert_eq!(command.rc_control().qy.control_type, ControlType::Angle);
        assert!((command.rc_control().qx.value - 0.393).abs() < 1e-6);
        assert!((command.rc_control().qy.value + 0.393).abs() < 1e-6);
    }

    #[test]
    fn offboard_passthrough_channels_survive_muxing() {
        let params = Params::new();
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModePassThrough,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                fx: 0.0,
                fy: 0.0,
                fz: 0.5,
                passthrough: [0.6, 0.7, 0.8, 0.9],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);

        command.run(1000, &params, &mut rc, &mut state);

        let passthrough = command.combined_control().passthrough;
        for (channel, expected) in passthrough.iter().zip([0.6, 0.7, 0.8, 0.9]) {
            assert!((channel.value - expected).abs() < 1e-6);
        }
        assert!(passthrough.iter().all(|channel| channel.active));
        assert!(
            passthrough
                .iter()
                .all(|channel| channel.control_type == ControlType::Passthrough)
        );
    }

    #[test]
    fn offboard_passthrough_keeps_ned_thrust_sign_for_mixer_commands() {
        let params = Params::new();
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);

        command.set_new_offboard_command(
            1_000_000,
            &OffboardControlMsg {
                mode: OffboardControlMode::ModePassThrough,
                ignore: OffboardControlIgnore::empty(),
                qx: 0.01,
                qy: -0.02,
                qz: 0.03,
                fx: 0.0,
                fy: 0.0,
                fz: -25.0,
                passthrough: [0.0; 4],
            },
            &params,
        );
        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);

        command.run(1000, &params, &mut rc, &mut state);

        let combined = command.combined_control();
        assert_eq!(combined.fz.control_type, ControlType::Passthrough);
        assert_eq!(combined.fz.value, -25.0);
        assert_eq!(combined.qx.control_type, ControlType::Passthrough);
        assert_eq!(combined.qx.value, 0.01);
    }

    #[test]
    fn failsafe_commands_match_rosflight_channel_types() {
        let command = CommandManager::new();

        assert_eq!(
            command.multirotor_failsafe_command.fx.control_type,
            ControlType::Throttle
        );
        assert_eq!(
            command.multirotor_failsafe_command.fy.control_type,
            ControlType::Throttle
        );
        assert_eq!(
            command.multirotor_failsafe_command.fz.control_type,
            ControlType::Throttle
        );
        assert_eq!(
            command.multirotor_failsafe_command.qx.control_type,
            ControlType::Angle
        );
        assert_eq!(
            command.multirotor_failsafe_command.qy.control_type,
            ControlType::Angle
        );
        assert_eq!(
            command.multirotor_failsafe_command.qz.control_type,
            ControlType::Rate
        );
        assert!(command.multirotor_failsafe_command.fx.active);
        assert!(command.multirotor_failsafe_command.fy.active);
        assert!(command.multirotor_failsafe_command.fz.active);

        assert!(command.fixedwing_failsafe_command.fx.active);
        assert!(command.fixedwing_failsafe_command.fy.active);
        assert!(command.fixedwing_failsafe_command.fz.active);
        assert_eq!(
            command.fixedwing_failsafe_command.fz.control_type,
            ControlType::Passthrough
        );
    }

    #[test]
    fn fixedwing_rc_command_uses_passthrough_channel_types() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
        let mut state = StateManager::new();
        let mut command = CommandManager::new();
        let mut rc = initialized_rc(&params);

        receive_rc(&mut rc, &params, &mut state, [0.5; RC_PACKET_CHANNELS]);
        command.interpret_rc(&rc, &params, None);

        let rc_control = command.rc_control();
        assert_eq!(rc_control.qx.control_type, ControlType::Passthrough);
        assert_eq!(rc_control.qy.control_type, ControlType::Passthrough);
        assert_eq!(rc_control.qz.control_type, ControlType::Passthrough);
        assert_eq!(rc_control.fx.control_type, ControlType::Passthrough);
        assert_eq!(rc_control.fy.control_type, ControlType::Passthrough);
        assert_eq!(rc_control.fz.control_type, ControlType::Passthrough);
    }

    #[test]
    fn fixedwing_failsafe_accepts_passthrough_throttle_outside_multirotor_range() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(1.5));
        let mut state = StateManager::new();
        let mut command = CommandManager::new();

        command.update_failsafe_config(&params, &mut state);

        assert!(!state.get_errors().contains(ErrorFlag::INVALID_FAILSAFE));
        assert_eq!(
            command.fixedwing_failsafe_command.fz.control_type,
            ControlType::Passthrough
        );
        assert_eq!(command.fixedwing_failsafe_command.fz.value, 0.0);
    }
}
