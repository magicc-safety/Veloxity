// /**
// ******************************************************************************
// * File     : params.rs
// * Date     : May 8, 2025
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************
// **/
use super::board::Board;
mod param_types;
pub use param_types::*;

pub struct Params {
    baud_rate: BaudRate,
    serial_device: SerialDevice,
    system_id: SystemId,
    max_command: MaxCommand,
    pid_roll_rate_p: PidRollRateP,
    pid_roll_rate_i: PidRollRateI,
    pid_roll_rate_d: PidRollRateD,
    pid_pitch_rate_p: PidPitchRateP,
    pid_pitch_rate_i: PidPitchRateI,
    pid_pitch_rate_d: PidPitchRateD,
    pid_yaw_rate_p: PidYawRateP,
    pid_yaw_rate_i: PidYawRateI,
    pid_yaw_rate_d: PidYawRateD,
    pid_roll_angle_p: PidRollAngleP,
    pid_roll_angle_i: PidRollAngleI,
    pid_roll_angle_d: PidRollAngleD,
    pid_pitch_angle_p: PidPitchAngleP,
    pid_pitch_angle_i: PidPitchAngleI,
    pid_pitch_angle_d: PidPitchAngleD,
    x_eq_torque: XEqTorque,
    y_eq_torque: YEqTorque,
    z_eq_torque: ZEqTorque,
    pid_tau: PidTau,
    motor_pwm_send_rate: MotorPwmSendRate,
    motor_idle_throttle: MotorIdleThrottle,
    failsafe_throttle: FailsafeThrottle,
    spin_motors_when_armed: SpinMotorsWhenArmed,
    init_time: InitTime,
    filter_kp_acc: FilterKpAcc,
    filter_ki: FilterKi,
    filter_kp_ext: FilterKpExt,
    filter_accel_margin: FilterAccelMargin,
    filter_use_quad_int: FilterUseQuadInt,
    filter_use_mat_exp: FilterUseMatExp,
    filter_use_acc: FilterUseAcc,
    calibrate_gyro_on_arm: CalibrateGyroOnArm,
    gyro_xy_alpha: GyroXyAlpha,
    gyro_z_alpha: GyroZAlpha,
    acc_alpha: AccAlpha,
    gyro_x_bias: GyroXBias,
    gyro_y_bias: GyroYBias,
    gyro_z_bias: GyroZBias,
    acc_x_bias: AccXBias,
    acc_y_bias: AccYBias,
    acc_z_bias: AccZBias,
    acc_x_temp_comp: AccXTempComp,
    acc_y_temp_comp: AccYTempComp,
    acc_z_temp_comp: AccZTempComp,
    mag_a11_comp: MagA11Comp,
    mag_a12_comp: MagA12Comp,
    mag_a13_comp: MagA13Comp,
    mag_a21_comp: MagA21Comp,
    mag_a22_comp: MagA22Comp,
    mag_a23_comp: MagA23Comp,
    mag_a31_comp: MagA31Comp,
    mag_a32_comp: MagA32Comp,
    mag_a33_comp: MagA33Comp,
    mag_x_bias: MagXBias,
    mag_y_bias: MagYBias,
    mag_z_bias: MagZBias,
    baro_bias: BaroBias,
    ground_level: GroundLevel,
    diff_press_bias: DiffPressBias,
    rc_type: RcType,
    rc_x_channel: RcXChannel,
    rc_y_channel: RcYChannel,
    rc_z_channel: RcZChannel,
    rc_f_channel: RcFChannel,
    rc_attitude_override_channel: RcAttitudeOverrideChannel,
    rc_throttle_override_channel: RcThrottleOverrideChannel,
    rc_att_control_type_channel: RcAttControlTypeChannel,
    rc_arm_channel: RcArmChannel,
    rc_num_channels: RcNumChannels,
    rc_switch_5_direction: RcSwitch5Direction,
    rc_switch_6_direction: RcSwitch6Direction,
    rc_switch_7_direction: RcSwitch7Direction,
    rc_switch_8_direction: RcSwitch8Direction,
    rc_override_deviation: RcOverrideDeviation,
    override_lag_time: OverrideLagTime,
    rc_override_take_min_throttle: RcOverrideTakeMinThrottle,
    rc_attitude_mode: RcAttitudeMode,
    rc_max_roll: RcMaxRoll,
    rc_max_pitch: RcMaxPitch,
    rc_max_rollrate: RcMaxRollRate,
    rc_max_pitchrate: RcMaxPitchRate,
    rc_max_yawrate: RcMaxYawRate,
    mixer: Mixer,
    fixed_wing: FixedWing,
    elevator_reverse: ElevatorReverse,
    aileron_reverse: AileronReverse,
    rudder_reverse: RudderReverse,
    fc_roll: FcRoll,
    fc_pitch: FcPitch,
    fc_yaw: FcYaw,
    arm_threshold: ArmThreshold,
    offboard_timeout: OffboardTimeout,
    battery_voltage_multiplier: BatteryVoltageMultiplier,
    battery_current_multiplier: BatteryCurrentMultiplier,
    battery_voltage_alpha: BatteryVoltageAlpha,
    battery_current_alpha: BatteryCurrentAlpha,
}

impl Params {
    //************************************************
    //***************** Getters **********************
    //************************************************
    pub fn get_baud_rate(&self) -> &ParamValue {
        return &self.baud_rate.value;
    }

    pub fn get_serial_device(&self) -> &ParamValue {
        return &self.serial_device.value;
    }

    pub fn get_system_id(&self) -> &ParamValue {
        return &self.system_id.value;
    }

    pub fn get_max_command(&self) -> &ParamValue {
        return &self.max_command.value;
    }

    pub fn get_pid_roll_rate_p(&self) -> &ParamValue {
        return &self.pid_roll_rate_p.value;
    }

    pub fn get_pid_roll_rate_i(&self) -> &ParamValue {
        return &self.pid_roll_rate_i.value;
    }

    pub fn get_pid_roll_rate_d(&self) -> &ParamValue {
        return &self.pid_roll_rate_d.value;
    }

    pub fn get_pid_pitch_rate_p(&self) -> &ParamValue {
        return &self.pid_pitch_rate_p.value;
    }

    pub fn get_pid_pitch_rate_i(&self) -> &ParamValue {
        return &self.pid_pitch_rate_i.value;
    }

    pub fn get_pid_pitch_rate_d(&self) -> &ParamValue {
        return &self.pid_pitch_rate_d.value;
    }

    pub fn get_pid_yaw_rate_p(&self) -> &ParamValue {
        return &self.pid_yaw_rate_p.value;
    }

    pub fn get_pid_yaw_rate_i(&self) -> &ParamValue {
        return &self.pid_yaw_rate_i.value;
    }

    pub fn get_pid_yaw_rate_d(&self) -> &ParamValue {
        return &self.pid_yaw_rate_d.value;
    }

    pub fn get_pid_roll_angle_p(&self) -> &ParamValue {
        return &self.pid_roll_angle_p.value;
    }

    pub fn get_pid_roll_angle_i(&self) -> &ParamValue {
        return &self.pid_roll_angle_i.value;
    }

    pub fn get_pid_roll_angle_d(&self) -> &ParamValue {
        return &self.pid_roll_angle_d.value;
    }

    pub fn get_pid_pitch_angle_p(&self) -> &ParamValue {
        return &self.pid_pitch_angle_p.value;
    }

    pub fn get_pid_pitch_angle_i(&self) -> &ParamValue {
        return &self.pid_pitch_angle_i.value;
    }

    pub fn get_pid_pitch_angle_d(&self) -> &ParamValue {
        return &self.pid_pitch_angle_d.value;
    }

    pub fn get_x_eq_torque(&self) -> &ParamValue {
        return &self.x_eq_torque.value;
    }

    pub fn get_y_eq_torque(&self) -> &ParamValue {
        return &self.y_eq_torque.value;
    }

    pub fn get_z_eq_torque(&self) -> &ParamValue {
        return &self.z_eq_torque.value;
    }

    pub fn get_pid_tau(&self) -> &ParamValue {
        return &self.pid_tau.value;
    }

    pub fn get_motor_pwm_send_rate(&self) -> &ParamValue {
        return &self.motor_pwm_send_rate.value;
    }

    pub fn get_motor_idle_throttle(&self) -> &ParamValue {
        return &self.motor_idle_throttle.value;
    }

    pub fn get_failsafe_throttle(&self) -> &ParamValue {
        return &self.failsafe_throttle.value;
    }

    pub fn get_spin_motors_when_armed(&self) -> &ParamValue {
        return &self.spin_motors_when_armed.value;
    }

    pub fn get_init_time(&self) -> &ParamValue {
        return &self.init_time.value;
    }

    pub fn get_filter_kp_acc(&self) -> &ParamValue {
        return &self.filter_kp_acc.value;
    }

    pub fn get_filter_ki(&self) -> &ParamValue {
        return &self.filter_ki.value;
    }

    pub fn get_filter_kp_ext(&self) -> &ParamValue {
        return &self.filter_kp_ext.value;
    }

    pub fn get_filter_accel_margin(&self) -> &ParamValue {
        return &self.filter_accel_margin.value;
    }

    pub fn get_filter_use_quad_int(&self) -> &ParamValue {
        return &self.filter_use_quad_int.value;
    }

    pub fn get_filter_use_mat_exp(&self) -> &ParamValue {
        return &self.filter_use_mat_exp.value;
    }

    pub fn get_filter_use_acc(&self) -> &ParamValue {
        return &self.filter_use_acc.value;
    }

    pub fn get_calibrate_gyro_on_arm(&self) -> &ParamValue {
        return &self.calibrate_gyro_on_arm.value;
    }

    pub fn get_gyro_xy_alpha(&self) -> &ParamValue {
        return &self.gyro_xy_alpha.value;
    }

    pub fn get_gyro_z_alpha(&self) -> &ParamValue {
        return &self.gyro_z_alpha.value;
    }

    pub fn get_acc_alpha(&self) -> &ParamValue {
        return &self.acc_alpha.value;
    }

    pub fn get_gyro_x_bias(&self) -> &ParamValue {
        return &self.gyro_x_bias.value;
    }

    pub fn get_gyro_y_bias(&self) -> &ParamValue {
        return &self.gyro_y_bias.value;
    }

    pub fn get_gyro_z_bias(&self) -> &ParamValue {
        return &self.gyro_z_bias.value;
    }

    pub fn get_acc_x_bias(&self) -> &ParamValue {
        return &self.acc_x_bias.value;
    }

    pub fn get_acc_y_bias(&self) -> &ParamValue {
        return &self.acc_y_bias.value;
    }

    pub fn get_acc_z_bias(&self) -> &ParamValue {
        return &self.acc_z_bias.value;
    }

    pub fn get_acc_x_temp_comp(&self) -> &ParamValue {
        return &self.acc_x_temp_comp.value;
    }

    pub fn get_acc_y_temp_comp(&self) -> &ParamValue {
        return &self.acc_y_temp_comp.value;
    }

    pub fn get_acc_z_temp_comp(&self) -> &ParamValue {
        return &self.acc_z_temp_comp.value;
    }

    pub fn get_mag_a11_comp(&self) -> &ParamValue {
        return &self.mag_a11_comp.value;
    }

    pub fn get_mag_a12_comp(&self) -> &ParamValue {
        return &self.mag_a12_comp.value;
    }

    pub fn get_mag_a13_comp(&self) -> &ParamValue {
        return &self.mag_a13_comp.value;
    }

    pub fn get_mag_a21_comp(&self) -> &ParamValue {
        return &self.mag_a21_comp.value;
    }

    pub fn get_mag_a22_comp(&self) -> &ParamValue {
        return &self.mag_a22_comp.value;
    }

    pub fn get_mag_a23_comp(&self) -> &ParamValue {
        return &self.mag_a23_comp.value;
    }

    pub fn get_mag_a31_comp(&self) -> &ParamValue {
        return &self.mag_a31_comp.value;
    }

    pub fn get_mag_a32_comp(&self) -> &ParamValue {
        return &self.mag_a32_comp.value;
    }

    pub fn get_mag_a33_comp(&self) -> &ParamValue {
        return &self.mag_a33_comp.value;
    }

    pub fn get_mag_x_bias(&self) -> &ParamValue {
        return &self.mag_x_bias.value;
    }

    pub fn get_mag_y_bias(&self) -> &ParamValue {
        return &self.mag_y_bias.value;
    }

    pub fn get_mag_z_bias(&self) -> &ParamValue {
        return &self.mag_z_bias.value;
    }

    pub fn get_baro_bias(&self) -> &ParamValue {
        return &self.baro_bias.value;
    }

    pub fn get_ground_level(&self) -> &ParamValue {
        return &self.ground_level.value;
    }

    pub fn get_diff_press_bias(&self) -> &ParamValue {
        return &self.diff_press_bias.value;
    }

    pub fn get_rc_type(&self) -> &ParamValue {
        return &self.rc_type.value;
    }

    pub fn get_rc_x_channel(&self) -> &ParamValue {
        return &self.rc_x_channel.value;
    }

    pub fn get_rc_y_channel(&self) -> &ParamValue {
        return &self.rc_y_channel.value;
    }

    pub fn get_rc_z_channel(&self) -> &ParamValue {
        return &self.rc_z_channel.value;
    }

    pub fn get_rc_f_channel(&self) -> &ParamValue {
        return &self.rc_f_channel.value;
    }

    pub fn get_rc_attitude_override_channel(&self) -> &ParamValue {
        return &self.rc_attitude_override_channel.value;
    }

    pub fn get_rc_throttle_override_channel(&self) -> &ParamValue {
        return &self.rc_throttle_override_channel.value;
    }

    pub fn get_rc_att_control_type_channel(&self) -> &ParamValue {
        return &self.rc_att_control_type_channel.value;
    }

    pub fn get_rc_arm_channel(&self) -> &ParamValue {
        return &self.rc_arm_channel.value;
    }

    pub fn get_rc_num_channels(&self) -> &ParamValue {
        return &self.rc_num_channels.value;
    }

    pub fn get_rc_switch_5_direction(&self) -> &ParamValue {
        return &self.rc_switch_5_direction.value;
    }

    pub fn get_rc_switch_6_direction(&self) -> &ParamValue {
        return &self.rc_switch_6_direction.value;
    }

    pub fn get_rc_switch_7_direction(&self) -> &ParamValue {
        return &self.rc_switch_7_direction.value;
    }

    pub fn get_rc_switch_8_direction(&self) -> &ParamValue {
        return &self.rc_switch_8_direction.value;
    }

    pub fn get_rc_override_deviation(&self) -> &ParamValue {
        return &self.rc_override_deviation.value;
    }

    pub fn get_override_lag_time(&self) -> &ParamValue {
        return &self.override_lag_time.value;
    }

    pub fn get_rc_override_take_min_throttle(&self) -> &ParamValue {
        return &self.rc_override_take_min_throttle.value;
    }

    pub fn get_rc_attitude_mode(&self) -> &ParamValue {
        return &self.rc_attitude_mode.value;
    }

    pub fn get_rc_max_roll(&self) -> &ParamValue {
        return &self.rc_max_roll.value;
    }

    pub fn get_rc_max_pitch(&self) -> &ParamValue {
        return &self.rc_max_pitch.value;
    }

    pub fn get_rc_max_rollrate(&self) -> &ParamValue {
        return &self.rc_max_rollrate.value;
    }

    pub fn get_rc_max_pitchrate(&self) -> &ParamValue {
        return &self.rc_max_pitchrate.value;
    }

    pub fn get_rc_max_yawrate(&self) -> &ParamValue {
        return &self.rc_max_yawrate.value;
    }

    pub fn get_mixer(&self) -> &ParamValue {
        return &self.mixer.value;
    }

    pub fn get_fixed_wing(&self) -> &ParamValue {
        return &self.fixed_wing.value;
    }

    pub fn get_elevator_reverse(&self) -> &ParamValue {
        return &self.elevator_reverse.value;
    }

    pub fn get_aileron_reverse(&self) -> &ParamValue {
        return &self.aileron_reverse.value;
    }

    pub fn get_rudder_reverse(&self) -> &ParamValue {
        return &self.rudder_reverse.value;
    }

    pub fn get_fc_roll(&self) -> &ParamValue {
        return &self.fc_roll.value;
    }

    pub fn get_fc_pitch(&self) -> &ParamValue {
        return &self.fc_pitch.value;
    }

    pub fn get_fc_yaw(&self) -> &ParamValue {
        return &self.fc_yaw.value;
    }

    pub fn get_arm_threshold(&self) -> &ParamValue {
        return &self.arm_threshold.value;
    }

    pub fn get_offboard_timeout(&self) -> &ParamValue {
        return &self.offboard_timeout.value;
    }

    pub fn get_battery_voltage_multiplier(&self) -> &ParamValue {
        return &self.battery_voltage_multiplier.value;
    }

    pub fn get_battery_current_multiplier(&self) -> &ParamValue {
        return &self.battery_current_multiplier.value;
    }

    pub fn get_battery_voltage_alpha(&self) -> &ParamValue {
        return &self.battery_voltage_alpha.value;
    }

    pub fn get_battery_current_alpha(&self) -> &ParamValue {
        return &self.battery_current_alpha.value;
    }

    //************************************************
    //***************** Setters **********************
    //************************************************
    pub fn set_baud_rate<'a>(&mut self, input: <BaudRate as Callback>::Args<'a>) {
        self.baud_rate.set(input);
    }

    pub fn set_serial_device<'a>(&mut self, input: <SerialDevice as Callback>::Args<'a>) {
        self.serial_device.set(input);
    }

    pub fn set_system_id<'a>(&mut self, input: <SystemId as Callback>::Args<'a>) {
        self.system_id.set(input);
    }

    pub fn set_max_command<'a>(&mut self, input: <MaxCommand as Callback>::Args<'a>) {
        self.max_command.set(input);
    }

    pub fn set_pid_roll_rate_p<'a>(&mut self, input: <PidRollRateP as Callback>::Args<'a>) {
        self.pid_roll_rate_p.set(input);
    }

    pub fn set_pid_roll_rate_i<'a>(&mut self, input: <PidRollRateI as Callback>::Args<'a>) {
        self.pid_roll_rate_i.set(input);
    }

    pub fn set_pid_roll_rate_d<'a>(&mut self, input: <PidRollRateD as Callback>::Args<'a>) {
        self.pid_roll_rate_d.set(input);
    }

    pub fn set_pid_pitch_rate_p<'a>(&mut self, input: <PidPitchRateP as Callback>::Args<'a>) {
        self.pid_pitch_rate_p.set(input);
    }

    pub fn set_pid_pitch_rate_i<'a>(&mut self, input: <PidPitchRateI as Callback>::Args<'a>) {
        self.pid_pitch_rate_i.set(input);
    }

    pub fn set_pid_pitch_rate_d<'a>(&mut self, input: <PidPitchRateD as Callback>::Args<'a>) {
        self.pid_pitch_rate_d.set(input);
    }

    pub fn set_pid_yaw_rate_p<'a>(&mut self, input: <PidYawRateP as Callback>::Args<'a>) {
        self.pid_yaw_rate_p.set(input);
    }

    pub fn set_pid_yaw_rate_i<'a>(&mut self, input: <PidYawRateI as Callback>::Args<'a>) {
        self.pid_yaw_rate_i.set(input);
    }

    pub fn set_pid_yaw_rate_d<'a>(&mut self, input: <PidYawRateD as Callback>::Args<'a>) {
        self.pid_yaw_rate_d.set(input);
    }

    pub fn set_pid_roll_angle_p<'a>(&mut self, input: <PidRollAngleP as Callback>::Args<'a>) {
        self.pid_roll_angle_p.set(input);
    }

    pub fn set_pid_roll_angle_i<'a>(&mut self, input: <PidRollAngleI as Callback>::Args<'a>) {
        self.pid_roll_angle_i.set(input);
    }

    pub fn set_pid_roll_angle_d<'a>(&mut self, input: <PidRollAngleD as Callback>::Args<'a>) {
        self.pid_roll_angle_d.set(input);
    }

    pub fn set_pid_pitch_angle_p<'a>(&mut self, input: <PidPitchAngleP as Callback>::Args<'a>) {
        self.pid_pitch_angle_p.set(input);
    }

    pub fn set_pid_pitch_angle_i<'a>(&mut self, input: <PidPitchAngleI as Callback>::Args<'a>) {
        self.pid_pitch_angle_i.set(input);
    }

    pub fn set_pid_pitch_angle_d<'a>(&mut self, input: <PidPitchAngleD as Callback>::Args<'a>) {
        self.pid_pitch_angle_d.set(input);
    }

    pub fn set_x_eq_torque<'a>(&mut self, input: <XEqTorque as Callback>::Args<'a>) {
        self.x_eq_torque.set(input);
    }

    pub fn set_y_eq_torque<'a>(&mut self, input: <YEqTorque as Callback>::Args<'a>) {
        self.y_eq_torque.set(input);
    }

    pub fn set_z_eq_torque<'a>(&mut self, input: <ZEqTorque as Callback>::Args<'a>) {
        self.z_eq_torque.set(input);
    }

    pub fn set_pid_tau<'a>(&mut self, input: <PidTau as Callback>::Args<'a>) {
        self.pid_tau.set(input);
    }

    pub fn set_motor_pwm_send_rate<'a>(&mut self, input: <MotorPwmSendRate as Callback>::Args<'a>) {
        self.motor_pwm_send_rate.set(input);
    }

    pub fn set_motor_idle_throttle<'a>(
        &mut self,
        input: <MotorIdleThrottle as Callback>::Args<'a>,
    ) {
        self.motor_idle_throttle.set(input);
    }

    pub fn set_failsafe_throttle<'a>(&mut self, input: <FailsafeThrottle as Callback>::Args<'a>) {
        self.failsafe_throttle.set(input);
    }

    pub fn set_spin_motors_when_armed<'a>(
        &mut self,
        input: <SpinMotorsWhenArmed as Callback>::Args<'a>,
    ) {
        self.spin_motors_when_armed.set(input);
    }

    pub fn set_init_time<'a>(&mut self, input: <InitTime as Callback>::Args<'a>) {
        self.init_time.set(input);
    }

    pub fn set_filter_kp_acc<'a>(&mut self, input: <FilterKpAcc as Callback>::Args<'a>) {
        self.filter_kp_acc.set(input);
    }

    pub fn set_filter_ki<'a>(&mut self, input: <FilterKi as Callback>::Args<'a>) {
        self.filter_ki.set(input);
    }

    pub fn set_filter_kp_ext<'a>(&mut self, input: <FilterKpExt as Callback>::Args<'a>) {
        self.filter_kp_ext.set(input);
    }

    pub fn set_filter_accel_margin<'a>(
        &mut self,
        input: <FilterAccelMargin as Callback>::Args<'a>,
    ) {
        self.filter_accel_margin.set(input);
    }

    pub fn set_filter_use_quad_int<'a>(&mut self, input: <FilterUseQuadInt as Callback>::Args<'a>) {
        self.filter_use_quad_int.set(input);
    }

    pub fn set_filter_use_mat_exp<'a>(&mut self, input: <FilterUseMatExp as Callback>::Args<'a>) {
        self.filter_use_mat_exp.set(input);
    }

    pub fn set_filter_use_acc<'a>(&mut self, input: <FilterUseAcc as Callback>::Args<'a>) {
        self.filter_use_acc.set(input);
    }

    pub fn set_calibrate_gyro_on_arm<'a>(
        &mut self,
        input: <CalibrateGyroOnArm as Callback>::Args<'a>,
    ) {
        self.calibrate_gyro_on_arm.set(input);
    }

    pub fn set_gyro_xy_alpha<'a>(&mut self, input: <GyroXyAlpha as Callback>::Args<'a>) {
        self.gyro_xy_alpha.set(input);
    }

    pub fn set_gyro_z_alpha<'a>(&mut self, input: <GyroZAlpha as Callback>::Args<'a>) {
        self.gyro_z_alpha.set(input);
    }

    pub fn set_acc_alpha<'a>(&mut self, input: <AccAlpha as Callback>::Args<'a>) {
        self.acc_alpha.set(input);
    }

    pub fn set_gyro_x_bias<'a>(&mut self, input: <GyroXBias as Callback>::Args<'a>) {
        self.gyro_x_bias.set(input);
    }

    pub fn set_gyro_y_bias<'a>(&mut self, input: <GyroYBias as Callback>::Args<'a>) {
        self.gyro_y_bias.set(input);
    }

    pub fn set_gyro_z_bias<'a>(&mut self, input: <GyroZBias as Callback>::Args<'a>) {
        self.gyro_z_bias.set(input);
    }

    pub fn set_acc_x_bias<'a>(&mut self, input: <AccXBias as Callback>::Args<'a>) {
        self.acc_x_bias.set(input);
    }

    pub fn set_acc_y_bias<'a>(&mut self, input: <AccYBias as Callback>::Args<'a>) {
        self.acc_y_bias.set(input);
    }

    pub fn set_acc_z_bias<'a>(&mut self, input: <AccZBias as Callback>::Args<'a>) {
        self.acc_z_bias.set(input);
    }

    pub fn set_acc_x_temp_comp<'a>(&mut self, input: <AccXTempComp as Callback>::Args<'a>) {
        self.acc_x_temp_comp.set(input);
    }

    pub fn set_acc_y_temp_comp<'a>(&mut self, input: <AccYTempComp as Callback>::Args<'a>) {
        self.acc_y_temp_comp.set(input);
    }

    pub fn set_acc_z_temp_comp<'a>(&mut self, input: <AccZTempComp as Callback>::Args<'a>) {
        self.acc_z_temp_comp.set(input);
    }

    pub fn set_mag_a11_comp<'a>(&mut self, input: <MagA11Comp as Callback>::Args<'a>) {
        self.mag_a11_comp.set(input);
    }

    pub fn set_mag_a12_comp<'a>(&mut self, input: <MagA12Comp as Callback>::Args<'a>) {
        self.mag_a12_comp.set(input);
    }

    pub fn set_mag_a13_comp<'a>(&mut self, input: <MagA13Comp as Callback>::Args<'a>) {
        self.mag_a13_comp.set(input);
    }

    pub fn set_mag_a21_comp<'a>(&mut self, input: <MagA21Comp as Callback>::Args<'a>) {
        self.mag_a21_comp.set(input);
    }

    pub fn set_mag_a22_comp<'a>(&mut self, input: <MagA22Comp as Callback>::Args<'a>) {
        self.mag_a22_comp.set(input);
    }

    pub fn set_mag_a23_comp<'a>(&mut self, input: <MagA23Comp as Callback>::Args<'a>) {
        self.mag_a23_comp.set(input);
    }

    pub fn set_mag_a31_comp<'a>(&mut self, input: <MagA31Comp as Callback>::Args<'a>) {
        self.mag_a31_comp.set(input);
    }

    pub fn set_mag_a32_comp<'a>(&mut self, input: <MagA32Comp as Callback>::Args<'a>) {
        self.mag_a32_comp.set(input);
    }

    pub fn set_mag_a33_comp<'a>(&mut self, input: <MagA33Comp as Callback>::Args<'a>) {
        self.mag_a33_comp.set(input);
    }

    pub fn set_mag_x_bias<'a>(&mut self, input: <MagXBias as Callback>::Args<'a>) {
        self.mag_x_bias.set(input);
    }

    pub fn set_mag_y_bias<'a>(&mut self, input: <MagYBias as Callback>::Args<'a>) {
        self.mag_y_bias.set(input);
    }

    pub fn set_mag_z_bias<'a>(&mut self, input: <MagZBias as Callback>::Args<'a>) {
        self.mag_z_bias.set(input);
    }

    pub fn set_baro_bias<'a>(&mut self, input: <BaroBias as Callback>::Args<'a>) {
        self.baro_bias.set(input);
    }

    pub fn set_ground_level<'a>(&mut self, input: <GroundLevel as Callback>::Args<'a>) {
        self.ground_level.set(input);
    }

    pub fn set_diff_press_bias<'a>(&mut self, input: <DiffPressBias as Callback>::Args<'a>) {
        self.diff_press_bias.set(input);
    }

    pub fn set_rc_type<'a>(&mut self, input: <RcType as Callback>::Args<'a>) {
        self.rc_type.set(input);
    }

    pub fn set_rc_x_channel<'a>(&mut self, input: <RcXChannel as Callback>::Args<'a>) {
        self.rc_x_channel.set(input);
    }

    pub fn set_rc_y_channel<'a>(&mut self, input: <RcYChannel as Callback>::Args<'a>) {
        self.rc_y_channel.set(input);
    }

    pub fn set_rc_z_channel<'a>(&mut self, input: <RcZChannel as Callback>::Args<'a>) {
        self.rc_z_channel.set(input);
    }

    pub fn set_rc_f_channel<'a>(&mut self, input: <RcFChannel as Callback>::Args<'a>) {
        self.rc_f_channel.set(input);
    }

    pub fn set_rc_attitude_override_channel<'a>(
        &mut self,
        input: <RcAttitudeOverrideChannel as Callback>::Args<'a>,
    ) {
        self.rc_attitude_override_channel.set(input);
    }

    pub fn set_rc_throttle_override_channel<'a>(
        &mut self,
        input: <RcThrottleOverrideChannel as Callback>::Args<'a>,
    ) {
        self.rc_throttle_override_channel.set(input);
    }

    pub fn set_rc_att_control_type_channel<'a>(
        &mut self,
        input: <RcAttControlTypeChannel as Callback>::Args<'a>,
    ) {
        self.rc_att_control_type_channel.set(input);
    }

    pub fn set_rc_arm_channel<'a>(&mut self, input: <RcArmChannel as Callback>::Args<'a>) {
        self.rc_arm_channel.set(input);
    }

    pub fn set_rc_num_channels<'a>(&mut self, input: <RcNumChannels as Callback>::Args<'a>) {
        self.rc_num_channels.set(input);
    }

    pub fn set_rc_switch_5_direction<'a>(
        &mut self,
        input: <RcSwitch5Direction as Callback>::Args<'a>,
    ) {
        self.rc_switch_5_direction.set(input);
    }

    pub fn set_rc_switch_6_direction<'a>(
        &mut self,
        input: <RcSwitch6Direction as Callback>::Args<'a>,
    ) {
        self.rc_switch_6_direction.set(input);
    }

    pub fn set_rc_switch_7_direction<'a>(
        &mut self,
        input: <RcSwitch7Direction as Callback>::Args<'a>,
    ) {
        self.rc_switch_7_direction.set(input);
    }

    pub fn set_rc_switch_8_direction<'a>(
        &mut self,
        input: <RcSwitch8Direction as Callback>::Args<'a>,
    ) {
        self.rc_switch_8_direction.set(input);
    }

    pub fn set_rc_override_deviation<'a>(
        &mut self,
        input: <RcOverrideDeviation as Callback>::Args<'a>,
    ) {
        self.rc_override_deviation.set(input);
    }

    pub fn set_override_lag_time<'a>(&mut self, input: <OverrideLagTime as Callback>::Args<'a>) {
        self.override_lag_time.set(input);
    }

    pub fn set_rc_override_take_min_throttle<'a>(
        &mut self,
        input: <RcOverrideTakeMinThrottle as Callback>::Args<'a>,
    ) {
        self.rc_override_take_min_throttle.set(input);
    }

    pub fn set_rc_attitude_mode<'a>(&mut self, input: <RcAttitudeMode as Callback>::Args<'a>) {
        self.rc_attitude_mode.set(input);
    }

    pub fn set_rc_max_roll<'a>(&mut self, input: <RcMaxRoll as Callback>::Args<'a>) {
        self.rc_max_roll.set(input);
    }

    pub fn set_rc_max_pitch<'a>(&mut self, input: <RcMaxPitch as Callback>::Args<'a>) {
        self.rc_max_pitch.set(input);
    }

    pub fn set_rc_max_rollrate<'a>(&mut self, input: <RcMaxRollRate as Callback>::Args<'a>) {
        self.rc_max_rollrate.set(input);
    }

    pub fn set_rc_max_pitchrate<'a>(&mut self, input: <RcMaxPitchRate as Callback>::Args<'a>) {
        self.rc_max_pitchrate.set(input);
    }

    pub fn set_rc_max_yawrate<'a>(&mut self, input: <RcMaxYawRate as Callback>::Args<'a>) {
        self.rc_max_yawrate.set(input);
    }

    pub fn set_mixer<'a>(&mut self, input: <Mixer as Callback>::Args<'a>) {
        self.mixer.set(input);
    }

    pub fn set_fixed_wing<'a>(&mut self, input: <FixedWing as Callback>::Args<'a>) {
        self.fixed_wing.set(input);
    }

    pub fn set_elevator_reverse<'a>(&mut self, input: <ElevatorReverse as Callback>::Args<'a>) {
        self.elevator_reverse.set(input);
    }

    pub fn set_aileron_reverse<'a>(&mut self, input: <AileronReverse as Callback>::Args<'a>) {
        self.aileron_reverse.set(input);
    }

    pub fn set_rudder_reverse<'a>(&mut self, input: <RudderReverse as Callback>::Args<'a>) {
        self.rudder_reverse.set(input);
    }

    pub fn set_fc_roll<'a>(&mut self, input: <FcRoll as Callback>::Args<'a>) {
        self.fc_roll.set(input);
    }

    pub fn set_fc_pitch<'a>(&mut self, input: <FcPitch as Callback>::Args<'a>) {
        self.fc_pitch.set(input);
    }

    pub fn set_fc_yaw<'a>(&mut self, input: <FcYaw as Callback>::Args<'a>) {
        self.fc_yaw.set(input);
    }

    pub fn set_arm_threshold<'a>(&mut self, input: <ArmThreshold as Callback>::Args<'a>) {
        self.arm_threshold.set(input);
    }

    pub fn set_offboard_timeout<'a>(&mut self, input: <OffboardTimeout as Callback>::Args<'a>) {
        self.offboard_timeout.set(input);
    }

    pub fn set_battery_voltage_multiplier<'a>(
        &mut self,
        input: <BatteryVoltageMultiplier as Callback>::Args<'a>,
    ) {
        self.battery_voltage_multiplier.set(input);
    }

    pub fn set_battery_current_multiplier<'a>(
        &mut self,
        input: <BatteryCurrentMultiplier as Callback>::Args<'a>,
    ) {
        self.battery_current_multiplier.set(input);
    }

    pub fn set_battery_voltage_alpha<'a>(
        &mut self,
        input: <BatteryVoltageAlpha as Callback>::Args<'a>,
    ) {
        self.battery_voltage_alpha.set(input);
    }

    pub fn set_battery_current_alpha<'a>(
        &mut self,
        input: <BatteryCurrentAlpha as Callback>::Args<'a>,
    ) {
        self.battery_current_alpha.set(input);
    }

    pub fn new() -> Self {
        Params {
            baud_rate: BaudRate {
                value: ParamValue::Int(0),
            },
            serial_device: SerialDevice {
                value: ParamValue::Int(0),
            },
            system_id: SystemId {
                value: ParamValue::Int(0),
            },
            max_command: MaxCommand {
                value: ParamValue::Int(0),
            },
            pid_roll_rate_p: PidRollRateP {
                value: ParamValue::Float(0.0),
            },
            pid_roll_rate_i: PidRollRateI {
                value: ParamValue::Float(0.0),
            },
            pid_roll_rate_d: PidRollRateD {
                value: ParamValue::Float(0.0),
            },
            pid_pitch_rate_p: PidPitchRateP {
                value: ParamValue::Float(0.0),
            },
            pid_pitch_rate_i: PidPitchRateI {
                value: ParamValue::Float(0.0),
            },
            pid_pitch_rate_d: PidPitchRateD {
                value: ParamValue::Float(0.0),
            },
            pid_yaw_rate_p: PidYawRateP {
                value: ParamValue::Float(0.0),
            },
            pid_yaw_rate_i: PidYawRateI {
                value: ParamValue::Float(0.0),
            },
            pid_yaw_rate_d: PidYawRateD {
                value: ParamValue::Float(0.0),
            },
            pid_roll_angle_p: PidRollAngleP {
                value: ParamValue::Float(0.0),
            },
            pid_roll_angle_i: PidRollAngleI {
                value: ParamValue::Float(0.0),
            },
            pid_roll_angle_d: PidRollAngleD {
                value: ParamValue::Float(0.0),
            },
            pid_pitch_angle_p: PidPitchAngleP {
                value: ParamValue::Float(0.0),
            },
            pid_pitch_angle_i: PidPitchAngleI {
                value: ParamValue::Float(0.0),
            },
            pid_pitch_angle_d: PidPitchAngleD {
                value: ParamValue::Float(0.0),
            },
            x_eq_torque: XEqTorque {
                value: ParamValue::Float(0.0),
            },
            y_eq_torque: YEqTorque {
                value: ParamValue::Float(0.0),
            },
            z_eq_torque: ZEqTorque {
                value: ParamValue::Float(0.0),
            },
            pid_tau: PidTau {
                value: ParamValue::Float(0.0),
            },
            motor_pwm_send_rate: MotorPwmSendRate {
                value: ParamValue::Int(0),
            },
            motor_idle_throttle: MotorIdleThrottle {
                value: ParamValue::Float(0.0),
            },
            failsafe_throttle: FailsafeThrottle {
                value: ParamValue::Float(0.0),
            },
            spin_motors_when_armed: SpinMotorsWhenArmed {
                value: ParamValue::Bool(false),
            },
            init_time: InitTime {
                value: ParamValue::Float(0.0),
            },
            filter_kp_acc: FilterKpAcc {
                value: ParamValue::Float(0.0),
            },
            filter_ki: FilterKi {
                value: ParamValue::Float(0.0),
            },
            filter_kp_ext: FilterKpExt {
                value: ParamValue::Float(0.0),
            },
            filter_accel_margin: FilterAccelMargin {
                value: ParamValue::Float(0.0),
            },
            filter_use_quad_int: FilterUseQuadInt {
                value: ParamValue::Bool(false),
            },
            filter_use_mat_exp: FilterUseMatExp {
                value: ParamValue::Bool(false),
            },
            filter_use_acc: FilterUseAcc {
                value: ParamValue::Bool(false),
            },
            calibrate_gyro_on_arm: CalibrateGyroOnArm {
                value: ParamValue::Bool(false),
            },
            gyro_xy_alpha: GyroXyAlpha {
                value: ParamValue::Float(0.0),
            },
            gyro_z_alpha: GyroZAlpha {
                value: ParamValue::Float(0.0),
            },
            acc_alpha: AccAlpha {
                value: ParamValue::Float(0.0),
            },
            gyro_x_bias: GyroXBias {
                value: ParamValue::Float(0.0),
            },
            gyro_y_bias: GyroYBias {
                value: ParamValue::Float(0.0),
            },
            gyro_z_bias: GyroZBias {
                value: ParamValue::Float(0.0),
            },
            acc_x_bias: AccXBias {
                value: ParamValue::Float(0.0),
            },
            acc_y_bias: AccYBias {
                value: ParamValue::Float(0.0),
            },
            acc_z_bias: AccZBias {
                value: ParamValue::Float(0.0),
            },
            acc_x_temp_comp: AccXTempComp {
                value: ParamValue::Float(0.0),
            },
            acc_y_temp_comp: AccYTempComp {
                value: ParamValue::Float(0.0),
            },
            acc_z_temp_comp: AccZTempComp {
                value: ParamValue::Float(0.0),
            },
            mag_a11_comp: MagA11Comp {
                value: ParamValue::Float(0.0),
            },
            mag_a12_comp: MagA12Comp {
                value: ParamValue::Float(0.0),
            },
            mag_a13_comp: MagA13Comp {
                value: ParamValue::Float(0.0),
            },
            mag_a21_comp: MagA21Comp {
                value: ParamValue::Float(0.0),
            },
            mag_a22_comp: MagA22Comp {
                value: ParamValue::Float(0.0),
            },
            mag_a23_comp: MagA23Comp {
                value: ParamValue::Float(0.0),
            },
            mag_a31_comp: MagA31Comp {
                value: ParamValue::Float(0.0),
            },
            mag_a32_comp: MagA32Comp {
                value: ParamValue::Float(0.0),
            },
            mag_a33_comp: MagA33Comp {
                value: ParamValue::Float(0.0),
            },
            mag_x_bias: MagXBias {
                value: ParamValue::Float(0.0),
            },
            mag_y_bias: MagYBias {
                value: ParamValue::Float(0.0),
            },
            mag_z_bias: MagZBias {
                value: ParamValue::Float(0.0),
            },
            baro_bias: BaroBias {
                value: ParamValue::Float(0.0),
            },
            ground_level: GroundLevel {
                value: ParamValue::Float(0.0),
            },
            diff_press_bias: DiffPressBias {
                value: ParamValue::Float(0.0),
            },
            rc_type: RcType {
                value: ParamValue::Int(0),
            },
            rc_x_channel: RcXChannel {
                value: ParamValue::Int(0),
            },
            rc_y_channel: RcYChannel {
                value: ParamValue::Int(0),
            },
            rc_z_channel: RcZChannel {
                value: ParamValue::Int(0),
            },
            rc_f_channel: RcFChannel {
                value: ParamValue::Int(0),
            },
            rc_attitude_override_channel: RcAttitudeOverrideChannel {
                value: ParamValue::Int(0),
            },
            rc_throttle_override_channel: RcThrottleOverrideChannel {
                value: ParamValue::Int(0),
            },
            rc_att_control_type_channel: RcAttControlTypeChannel {
                value: ParamValue::Int(0),
            },
            rc_arm_channel: RcArmChannel {
                value: ParamValue::Int(0),
            },
            rc_num_channels: RcNumChannels {
                value: ParamValue::Int(0),
            },
            rc_switch_5_direction: RcSwitch5Direction {
                value: ParamValue::Int(0),
            },
            rc_switch_6_direction: RcSwitch6Direction {
                value: ParamValue::Int(0),
            },
            rc_switch_7_direction: RcSwitch7Direction {
                value: ParamValue::Int(0),
            },
            rc_switch_8_direction: RcSwitch8Direction {
                value: ParamValue::Int(0),
            },
            rc_override_deviation: RcOverrideDeviation {
                value: ParamValue::Float(0.0),
            },
            override_lag_time: OverrideLagTime {
                value: ParamValue::Float(0.0),
            },
            rc_override_take_min_throttle: RcOverrideTakeMinThrottle {
                value: ParamValue::Bool(false),
            },
            rc_attitude_mode: RcAttitudeMode {
                value: ParamValue::Int(0),
            },
            rc_max_roll: RcMaxRoll {
                value: ParamValue::Float(0.0),
            },
            rc_max_pitch: RcMaxPitch {
                value: ParamValue::Float(0.0),
            },
            rc_max_rollrate: RcMaxRollRate {
                value: ParamValue::Float(0.0),
            },
            rc_max_pitchrate: RcMaxPitchRate {
                value: ParamValue::Float(0.0),
            },
            rc_max_yawrate: RcMaxYawRate {
                value: ParamValue::Float(0.0),
            },
            mixer: Mixer {
                value: ParamValue::Int(0),
            },
            fixed_wing: FixedWing {
                value: ParamValue::Bool(false),
            },
            elevator_reverse: ElevatorReverse {
                value: ParamValue::Bool(false),
            },
            aileron_reverse: AileronReverse {
                value: ParamValue::Bool(false),
            },
            rudder_reverse: RudderReverse {
                value: ParamValue::Bool(false),
            },
            fc_roll: FcRoll {
                value: ParamValue::Float(0.0),
            },
            fc_pitch: FcPitch {
                value: ParamValue::Float(0.0),
            },
            fc_yaw: FcYaw {
                value: ParamValue::Float(0.0),
            },
            arm_threshold: ArmThreshold {
                value: ParamValue::Float(0.0),
            },
            offboard_timeout: OffboardTimeout {
                value: ParamValue::Float(0.0),
            },
            battery_voltage_multiplier: BatteryVoltageMultiplier {
                value: ParamValue::Float(0.0),
            },
            battery_current_multiplier: BatteryCurrentMultiplier {
                value: ParamValue::Float(0.0),
            },
            battery_voltage_alpha: BatteryVoltageAlpha {
                value: ParamValue::Float(0.0),
            },
            battery_current_alpha: BatteryCurrentAlpha {
                value: ParamValue::Float(0.0),
            },
        }
    }

    pub fn set_defaults(&mut self) {
        // Hardware Configuration
        self.set_baud_rate((
            &mut TestStruct { val: 10, val2: 20 },
            ParamValue::Int(91600),
        ));
        self.set_serial_device(ParamValue::Int(0));

        // MAVLink Configuration
        self.set_system_id(ParamValue::Int(1));

        // Controller Configuration
        self.set_max_command(ParamValue::Float(0.100));

        // PID Rate Parameters
        self.set_pid_roll_rate_p(ParamValue::Float(0.070));
        self.set_pid_roll_rate_i(ParamValue::Float(0.000));
        self.set_pid_roll_rate_d(ParamValue::Float(0.000));
        self.set_pid_pitch_rate_p(ParamValue::Float(0.070));
        self.set_pid_pitch_rate_i(ParamValue::Float(0.000));
        self.set_pid_pitch_rate_d(ParamValue::Float(0.000));
        self.set_pid_yaw_rate_p(ParamValue::Float(0.250));
        self.set_pid_yaw_rate_i(ParamValue::Float(0.000));
        self.set_pid_yaw_rate_d(ParamValue::Float(0.000));

        // PID Angle Parameters
        self.set_pid_roll_angle_p(ParamValue::Float(0.150));
        self.set_pid_roll_angle_i(ParamValue::Float(0.000));
        self.set_pid_roll_angle_d(ParamValue::Float(0.050));
        self.set_pid_pitch_angle_p(ParamValue::Float(0.150));
        self.set_pid_pitch_angle_i(ParamValue::Float(0.000));
        self.set_pid_pitch_angle_d(ParamValue::Float(0.050));

        // Equilibrium Torque
        self.set_x_eq_torque(ParamValue::Float(0.000));
        self.set_y_eq_torque(ParamValue::Float(0.000));
        self.set_z_eq_torque(ParamValue::Float(0.000));
        self.set_pid_tau(ParamValue::Float(0.050));

        // PWM Configuration
        self.set_motor_pwm_send_rate(ParamValue::Int(0));
        self.set_motor_idle_throttle(ParamValue::Float(0.100));
        self.set_failsafe_throttle(ParamValue::Float(-1.00));
        self.set_spin_motors_when_armed(ParamValue::Bool(true));

        // Estimator Configuration
        self.set_init_time(ParamValue::Int(3000));
        self.set_filter_kp_acc(ParamValue::Float(0.500));
        self.set_filter_ki(ParamValue::Float(0.010));
        self.set_filter_kp_ext(ParamValue::Float(2.500));
        self.set_filter_accel_margin(ParamValue::Float(0.100));
        self.set_filter_use_quad_int(ParamValue::Bool(true));
        self.set_filter_use_mat_exp(ParamValue::Bool(true));
        self.set_filter_use_acc(ParamValue::Bool(true));
        self.set_calibrate_gyro_on_arm(ParamValue::Bool(false));

        // Gyro and Acc Alpha
        self.set_gyro_xy_alpha(ParamValue::Float(0.300));
        self.set_gyro_z_alpha(ParamValue::Float(0.300));
        self.set_acc_alpha(ParamValue::Float(0.500));

        // Bias Parameters
        self.set_gyro_x_bias(ParamValue::Float(0.000));
        self.set_gyro_y_bias(ParamValue::Float(0.000));
        self.set_gyro_z_bias(ParamValue::Float(0.000));
        self.set_acc_x_bias(ParamValue::Float(0.000));
        self.set_acc_y_bias(ParamValue::Float(0.000));
        self.set_acc_z_bias(ParamValue::Float(0.000));

        // Temperature Compensation
        self.set_acc_x_temp_comp(ParamValue::Float(0.000));
        self.set_acc_y_temp_comp(ParamValue::Float(0.000));
        self.set_acc_z_temp_comp(ParamValue::Float(0.000));

        // Magnetometer Compensation
        self.set_mag_a11_comp(ParamValue::Float(1.000));
        self.set_mag_a12_comp(ParamValue::Float(0.000));
        self.set_mag_a13_comp(ParamValue::Float(0.000));
        self.set_mag_a21_comp(ParamValue::Float(0.000));
        self.set_mag_a22_comp(ParamValue::Float(1.000));
        self.set_mag_a23_comp(ParamValue::Float(0.000));
        self.set_mag_a31_comp(ParamValue::Float(0.000));
        self.set_mag_a32_comp(ParamValue::Float(0.000));
        self.set_mag_a33_comp(ParamValue::Float(1.000));

        // Magnetometer Bias
        self.set_mag_x_bias(ParamValue::Float(0.000));
        self.set_mag_y_bias(ParamValue::Float(0.000));
        self.set_mag_z_bias(ParamValue::Float(0.000));

        // Barometer and Pressure
        self.set_baro_bias(ParamValue::Float(0.000));
        self.set_ground_level(ParamValue::Float(1387.0));
        self.set_diff_press_bias(ParamValue::Float(0.000));

        // RC Configuration
        self.set_rc_type(ParamValue::Int(0));
        self.set_rc_x_channel(ParamValue::Int(0));
        self.set_rc_y_channel(ParamValue::Int(1));
        self.set_rc_z_channel(ParamValue::Int(3));
        self.set_rc_f_channel(ParamValue::Int(2));
        self.set_rc_attitude_override_channel(ParamValue::Int(4));
        self.set_rc_throttle_override_channel(ParamValue::Int(4));
        self.set_rc_att_control_type_channel(ParamValue::Int(-1));
        self.set_rc_arm_channel(ParamValue::Int(-1));
        self.set_rc_num_channels(ParamValue::Int(6));

        // RC Switch Directions
        self.set_rc_switch_5_direction(ParamValue::Int(1));
        self.set_rc_switch_6_direction(ParamValue::Int(1));
        self.set_rc_switch_7_direction(ParamValue::Int(1));
        self.set_rc_switch_8_direction(ParamValue::Int(1));

        // RC Override Parameters
        self.set_rc_override_deviation(ParamValue::Float(0.100));
        self.set_override_lag_time(ParamValue::Int(1000));
        self.set_rc_override_take_min_throttle(ParamValue::Bool(true));

        // RC Attitude Parameters
        self.set_rc_attitude_mode(ParamValue::Int(1));
        self.set_rc_max_roll(ParamValue::Float(0.786));
        self.set_rc_max_pitch(ParamValue::Float(0.786));
        self.set_rc_max_rollrate(ParamValue::Float(3.14159));
        self.set_rc_max_pitchrate(ParamValue::Float(3.14159));
        self.set_rc_max_yawrate(ParamValue::Float(1.507));

        // Frame Configuration
        self.set_mixer(ParamValue::Int(0));
        self.set_fixed_wing(ParamValue::Bool(false));
        self.set_elevator_reverse(ParamValue::Bool(false));
        self.set_aileron_reverse(ParamValue::Bool(false));
        self.set_rudder_reverse(ParamValue::Bool(false));

        // Frame Compensation
        self.set_fc_roll(ParamValue::Float(0.000));
        self.set_fc_pitch(ParamValue::Float(0.000));
        self.set_fc_yaw(ParamValue::Float(0.000));

        // Arming Setup
        self.set_arm_threshold(ParamValue::Float(0.150));

        // Offboard Control
        self.set_offboard_timeout(ParamValue::Int(100));

        // Battery Monitor
        self.set_battery_voltage_multiplier(ParamValue::Float(0.000));
        self.set_battery_current_multiplier(ParamValue::Float(0.000));
        self.set_battery_voltage_alpha(ParamValue::Float(0.995));
        self.set_battery_current_alpha(ParamValue::Float(0.995));
    }

    pub fn get_param(&self, param_name: &str) -> Option<&ParamValue> {
        match param_name {
            "BAUD_RATE" => Some(self.get_baud_rate()),
            "SERIAL_DEVICE" => Some(self.get_serial_device()),
            "SYSTEM_ID" => Some(self.get_system_id()),
            "MAX_COMMAND" => Some(self.get_max_command()),
            "PID_ROLL_RATE_P" => Some(self.get_pid_roll_rate_p()),
            "PID_ROLL_RATE_I" => Some(self.get_pid_roll_rate_i()),
            "PID_ROLL_RATE_D" => Some(self.get_pid_roll_rate_d()),
            "PID_PITCH_RATE_P" => Some(self.get_pid_pitch_rate_p()),
            "PID_PITCH_RATE_I" => Some(self.get_pid_pitch_rate_i()),
            "PID_PITCH_RATE_D" => Some(self.get_pid_pitch_rate_d()),
            "PID_YAW_RATE_P" => Some(self.get_pid_yaw_rate_p()),
            "PID_YAW_RATE_I" => Some(self.get_pid_yaw_rate_i()),
            "PID_YAW_RATE_D" => Some(self.get_pid_yaw_rate_d()),
            "PID_ROLL_ANGLE_P" => Some(self.get_pid_roll_angle_p()),
            "PID_ROLL_ANGLE_I" => Some(self.get_pid_roll_angle_i()),
            "PID_ROLL_ANGLE_D" => Some(self.get_pid_roll_angle_d()),
            "PID_PITCH_ANGLE_P" => Some(self.get_pid_pitch_angle_p()),
            "PID_PITCH_ANGLE_I" => Some(self.get_pid_pitch_angle_i()),
            "PID_PITCH_ANGLE_D" => Some(self.get_pid_pitch_angle_d()),
            "X_EQ_TORQUE" => Some(self.get_x_eq_torque()),
            "Y_EQ_TORQUE" => Some(self.get_y_eq_torque()),
            "Z_EQ_TORQUE" => Some(self.get_z_eq_torque()),
            "PID_TAU" => Some(self.get_pid_tau()),
            "MOTOR_PWM_UPDATE" => Some(self.get_motor_pwm_send_rate()),
            "MOTOR_IDLE_THR" => Some(self.get_motor_idle_throttle()),
            "FAILSAFE_THR" => Some(self.get_failsafe_throttle()),
            "ARM_SPIN_MOTORS" => Some(self.get_spin_motors_when_armed()),
            "FILTER_INIT_T" => Some(self.get_init_time()),
            "FILTER_KP_ACC" => Some(self.get_filter_kp_acc()),
            "FILTER_KI" => Some(self.get_filter_ki()),
            "FILTER_KP_EXT" => Some(self.get_filter_kp_ext()),
            "FILTER_ACCMARGIN" => Some(self.get_filter_accel_margin()),
            "FILTER_QUAD_INT" => Some(self.get_filter_use_quad_int()),
            "FILTER_MAT_EXP" => Some(self.get_filter_use_mat_exp()),
            "FILTER_USE_ACC" => Some(self.get_filter_use_acc()),
            "CAL_GYRO_ARM" => Some(self.get_calibrate_gyro_on_arm()),
            "GRYO_LPF_ALPHA" => Some(self.get_gyro_xy_alpha()),
            "GYRO_Z_LPF_ALPHA" => Some(self.get_gyro_z_alpha()),
            "ACC_LPF_ALPHA" => Some(self.get_acc_alpha()),
            "GYRO_X_BIAS" => Some(self.get_gyro_x_bias()),
            "GYRO_Y_BIAS" => Some(self.get_gyro_y_bias()),
            "GYRO_Z_BIAS" => Some(self.get_gyro_z_bias()),
            "ACC_X_BIAS" => Some(self.get_acc_x_bias()),
            "ACC_Y_BIAS" => Some(self.get_acc_y_bias()),
            "ACC_Z_BIAS" => Some(self.get_acc_z_bias()),
            "ACC_X_TEMP_COMP" => Some(self.get_acc_x_temp_comp()),
            "ACC_Y_TEMP_COMP" => Some(self.get_acc_y_temp_comp()),
            "ACC_Z_TEMP_COMP" => Some(self.get_acc_z_temp_comp()),
            "MAG_A11_COMP" => Some(self.get_mag_a11_comp()),
            "MAG_A12_COMP" => Some(self.get_mag_a12_comp()),
            "MAG_A13_COMP" => Some(self.get_mag_a13_comp()),
            "MAG_A21_COMP" => Some(self.get_mag_a21_comp()),
            "MAG_A22_COMP" => Some(self.get_mag_a22_comp()),
            "MAG_A23_COMP" => Some(self.get_mag_a23_comp()),
            "MAG_A31_COMP" => Some(self.get_mag_a31_comp()),
            "MAG_A32_COMP" => Some(self.get_mag_a32_comp()),
            "MAG_A33_COMP" => Some(self.get_mag_a33_comp()),
            "MAG_X_BIAS" => Some(self.get_mag_x_bias()),
            "MAG_Y_BIAS" => Some(self.get_mag_y_bias()),
            "MAG_Z_BIAS" => Some(self.get_mag_z_bias()),
            "BARO_BIAS" => Some(self.get_baro_bias()),
            "GROUND_LEVEL" => Some(self.get_ground_level()),
            "DIFF_PRESS_BIAS" => Some(self.get_diff_press_bias()),
            "RC_TYPE" => Some(self.get_rc_type()),
            "RC_X_CHN" => Some(self.get_rc_x_channel()),
            "RC_Y_CHN" => Some(self.get_rc_y_channel()),
            "RC_Z_CHN" => Some(self.get_rc_z_channel()),
            "RC_F_CHN" => Some(self.get_rc_f_channel()),
            "RC_ATT_OVRD_CHN" => Some(self.get_rc_attitude_override_channel()),
            "RC_THR_OVRD_CHN" => Some(self.get_rc_throttle_override_channel()),
            "RC_ATT_CTRL_CHN" => Some(self.get_rc_att_control_type_channel()),
            "ARM_CHANNEL" => Some(self.get_rc_arm_channel()),
            "RC_NUM_CHN" => Some(self.get_rc_num_channels()),
            "SWITCH_5_DIR" => Some(self.get_rc_switch_5_direction()),
            "SWITCH_6_DIR" => Some(self.get_rc_switch_6_direction()),
            "SWITCH_7_DIR" => Some(self.get_rc_switch_7_direction()),
            "SWITCH_8_DIR" => Some(self.get_rc_switch_8_direction()),
            "RC_OVRD_DEV" => Some(self.get_rc_override_deviation()),
            "OVRD_LAG_TIME" => Some(self.get_override_lag_time()),
            "MIN_THROTTLE" => Some(self.get_rc_override_take_min_throttle()),
            "RC_ATT_MODE" => Some(self.get_rc_attitude_mode()),
            "RC_MAX_ROLL" => Some(self.get_rc_max_roll()),
            "RC_MAX_PITCH" => Some(self.get_rc_max_pitch()),
            "RC_MAX_ROLLRATE" => Some(self.get_rc_max_rollrate()),
            "RC_MAX_PITCHRATE" => Some(self.get_rc_max_pitchrate()),
            "RC_MAX_YAWRATE" => Some(self.get_rc_max_yawrate()),
            "MIXER" => Some(self.get_mixer()),
            "FIXED_WING" => Some(self.get_fixed_wing()),
            "ELEV_REV" => Some(self.get_elevator_reverse()),
            "AIL_REV" => Some(self.get_aileron_reverse()),
            "RUD_REV" => Some(self.get_rudder_reverse()),
            "FC_ROLL" => Some(self.get_fc_roll()),
            "FC_PITCH" => Some(self.get_fc_pitch()),
            "FC_YAW" => Some(self.get_fc_yaw()),
            "ARM_THRESHOLD" => Some(self.get_arm_threshold()),
            "OFFBOARD_TIMEOUT" => Some(self.get_offboard_timeout()),
            "BATT_VOLT_MULT" => Some(self.get_battery_voltage_multiplier()),
            "BATT_CURR_MULT" => Some(self.get_battery_current_multiplier()),
            "BATT_VOLT_ALPHA" => Some(self.get_battery_voltage_alpha()),
            "BATT_CURR_ALPHA" => Some(self.get_battery_current_alpha()),
            _ => None,
        }
    }

    pub fn init(&mut self, board: &mut dyn Board) {
        self.set_defaults();
        self.write(board);
    }

    pub fn write(&mut self, board: &mut dyn Board) -> bool {
        // if !board.memory_write(self) {
        //     return false;
        // }

        true
    }

    pub fn read(&mut self, board: &dyn Board) -> bool {
        // if !board.memory_read(self, p: &Params) {
        //     return false;
        // }

        true
    }
}

#[cfg(test)]
mod test_params {
    use super::*;

    #[test]
    fn test_update() {
        let mut p = Params::new();
        p.set_acc_alpha(ParamValue::Bool(true));
        assert_eq!(p.get_acc_alpha(), &ParamValue::Bool(true));
    }

    #[test]
    fn test_callback() {
        let mut p = Params::new();
        let mut x = TestStruct { val: 5, val2: 10 };

        p.set_baud_rate((&mut x, ParamValue::Float(20.0)));
    }

    #[test]
    fn test_lookup_string() {
        let mut p = Params::new();
        p.set_defaults();
        assert_eq!(p.get_param("BAUD_RATE").unwrap(), &ParamValue::Int(91600));
    }
}
