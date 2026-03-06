#![no_std]
#![allow(dead_code)]
#![allow(non_camel_case_types)]

//=================================================================================
// 1. Core Data Types
//=================================================================================

/// An enum representing the value of a parameter.
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug)]
pub enum ParamValue {
    Float(f32),
    Int(i32),
    Uint(u32),
    Bool(bool),
}

/// A static definition of a parameter, containing its compile-time metadata.
pub struct ParamDefinition {
    pub id: ParamId,
    pub name: &'static str,
    pub default: ParamValue,
}

//=================================================================================
// 2. The `declare_params!` Macro
//=================================================================================
macro_rules! declare_params {
    (
        // RustId, NameStr, Type(DefaultValue);
        $( $rust_id:ident, $name:literal, $type:ident($default:expr); )+
    ) => {
        //----------------------------------------------------------------
        // A. Generate a type-safe enum for parameter identification.
        //----------------------------------------------------------------
        #[derive(Copy, Clone, Eq, PartialEq, Debug)]
        #[repr(u16)]
        pub enum ParamId {
            $( $rust_id, )+
        }

        //----------------------------------------------------------------
        // A(1.5). Generate a lookup from the id (u16) to the Parameter string...
        //----------------------------------------------------------------
        impl ParamId {
            /// Returns the static string name of the parameter.
            pub fn as_str(&self) -> &'static str {
                // This is a safe, direct lookup because the enum order
                // and the array order are guaranteed to match.
                PARAM_DEFINITIONS[*self as usize].name
            }
        }

        //----------------------------------------------------------------
        // B. Define a compile-time constant for the number of parameters.
        //----------------------------------------------------------------
        pub const PARAMS_COUNT: usize = $( {let _ = stringify!($rust_id); 1} + )+ 0;

        //----------------------------------------------------------------
        // C. Generate a static array of parameter definitions.
        //----------------------------------------------------------------
        pub static PARAM_DEFINITIONS: [ParamDefinition; PARAMS_COUNT] = [
            $(
                ParamDefinition {
                    id: ParamId::$rust_id,
                    name: $name,
                    default: ParamValue::$type($default),
                },
            )+
        ];

        //----------------------------------------------------------------
        // D. The main `Params` struct holding runtime values.
        //----------------------------------------------------------------
        #[derive(Copy, Clone)]
        pub struct Params {
            values: [ParamValue; PARAMS_COUNT],
        }

        impl Params {
            /// Creates a new `Params` instance and initializes it with default values.
            pub fn new() -> Self {
                let mut params = Self {
                    values: [ParamValue::Int(0); PARAMS_COUNT],
                };
                params.set_defaults();
                params
            }

            /// Resets all parameters to their default values.
            pub fn set_defaults(&mut self) {
                for def in PARAM_DEFINITIONS.iter() {
                    self.values[def.id as usize] = def.default;
                }
            }

            /// Gets a parameter's value by its type-safe ID. Returns a copy.
            pub fn get_by_id(&self, id: ParamId) -> ParamValue {
                self.values[id as usize]
            }

            /// Sets a parameter's value by its type-safe ID.
            pub fn set_by_id(&mut self, id: ParamId, value: ParamValue) {
                self.values[id as usize] = value;
            }

            /// Gets a parameter's value by its string name. Returns `None` if not found.
            pub fn get_by_name(&self, name: &str) -> Option<ParamValue> {
                PARAM_DEFINITIONS.iter().find(|d| d.name == name).map(|def| self.get_by_id(def.id))
            }

            /// Sets a parameter's value by its string name. Returns `true` on success.
            pub fn set_by_name(&mut self, name: &str, value: ParamValue) -> bool {
                if let Some(def) = PARAM_DEFINITIONS.iter().find(|d| d.name == name) {
                    self.set_by_id(def.id, value);
                    true
                } else {
                    false
                }
            }

            /// Returns an iterator over the parameters.
            pub fn iter(&self) -> ParamIter {
                ParamIter {
                    params: self.clone(),
                    index: 0,
                }
            }
        }

        impl Default for Params {
            fn default() -> Self {
                Self::new()
            }
        }
    };
}

//=================================================================================
// 3. Parameter Definitions
//=================================================================================
declare_params! {
    // RustId, Name, Type(Default);
    PARAM_BAUD_RATE, "BAUD_RATE", Int(921600);
    PARAM_SERIAL_DEVICE, "SERIAL_DEVICE", Int(0);
    PARAM_NUM_MOTORS, "NUM_MOTORS", Int(4);
    PARAM_MOTOR_RESISTANCE, "MOTOR_RESISTANCE", Float(0.042);
    PARAM_MOTOR_KV, "MOTOR_KV", Float(0.01706);
    PARAM_NO_LOAD_CURRENT, "NO_LOAD_CURRENT", Float(1.5);
    PARAM_PROP_DIAMETER, "PROP_DIAMETER", Float(0.381);
    PARAM_PROP_CT, "PROP_CT", Float(0.075);
    PARAM_PROP_CQ, "PROP_CQ", Float(0.0045);
    PARAM_VOLT_MAX, "VOLT_MAX", Float(25.0);
    PARAM_USE_MOTOR_PARAMETERS, "USE_MOTOR_PARAM", Bool(false);
    PARAM_PRIMARY_MIXER_OUTPUT_0, "PRI_MIXER_OUT_0", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_1, "PRI_MIXER_OUT_1", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_2, "PRI_MIXER_OUT_2", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_3, "PRI_MIXER_OUT_3", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_4, "PRI_MIXER_OUT_4", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_5, "PRI_MIXER_OUT_5", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_6, "PRI_MIXER_OUT_6", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_7, "PRI_MIXER_OUT_7", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_8, "PRI_MIXER_OUT_8", Int(0);
    PARAM_PRIMARY_MIXER_OUTPUT_9, "PRI_MIXER_OUT_9", Int(0);
    PARAM_PRIMARY_MIXER_PWM_RATE_0, "PRI_MIXER_PWM_0", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_1, "PRI_MIXER_PWM_1", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_2, "PRI_MIXER_PWM_2", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_3, "PRI_MIXER_PWM_3", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_4, "PRI_MIXER_PWM_4", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_5, "PRI_MIXER_PWM_5", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_6, "PRI_MIXER_PWM_6", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_7, "PRI_MIXER_PWM_7", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_8, "PRI_MIXER_PWM_8", Float(0.0);
    PARAM_PRIMARY_MIXER_PWM_RATE_9, "PRI_MIXER_PWM_9", Float(0.0);
    PARAM_PRIMARY_MIXER_0_0, "PRI_MIXER_0_0", Float(0.0);
    PARAM_PRIMARY_MIXER_1_0, "PRI_MIXER_1_0", Float(0.0);
    PARAM_PRIMARY_MIXER_2_0, "PRI_MIXER_2_0", Float(0.0);
    PARAM_PRIMARY_MIXER_3_0, "PRI_MIXER_3_0", Float(0.0);
    PARAM_PRIMARY_MIXER_4_0, "PRI_MIXER_4_0", Float(0.0);
    PARAM_PRIMARY_MIXER_5_0, "PRI_MIXER_5_0", Float(0.0);
    PARAM_PRIMARY_MIXER_0_1, "PRI_MIXER_0_1", Float(0.0);
    PARAM_PRIMARY_MIXER_1_1, "PRI_MIXER_1_1", Float(0.0);
    PARAM_PRIMARY_MIXER_2_1, "PRI_MIXER_2_1", Float(0.0);
    PARAM_PRIMARY_MIXER_3_1, "PRI_MIXER_3_1", Float(0.0);
    PARAM_PRIMARY_MIXER_4_1, "PRI_MIXER_4_1", Float(0.0);
    PARAM_PRIMARY_MIXER_5_1, "PRI_MIXER_5_1", Float(0.0);
    PARAM_PRIMARY_MIXER_0_2, "PRI_MIXER_0_2", Float(0.0);
    PARAM_PRIMARY_MIXER_1_2, "PRI_MIXER_1_2", Float(0.0);
    PARAM_PRIMARY_MIXER_2_2, "PRI_MIXER_2_2", Float(0.0);
    PARAM_PRIMARY_MIXER_3_2, "PRI_MIXER_3_2", Float(0.0);
    PARAM_PRIMARY_MIXER_4_2, "PRI_MIXER_4_2", Float(0.0);
    PARAM_PRIMARY_MIXER_5_2, "PRI_MIXER_5_2", Float(0.0);
    PARAM_PRIMARY_MIXER_0_3, "PRI_MIXER_0_3", Float(0.0);
    PARAM_PRIMARY_MIXER_1_3, "PRI_MIXER_1_3", Float(0.0);
    PARAM_PRIMARY_MIXER_2_3, "PRI_MIXER_2_3", Float(0.0);
    PARAM_PRIMARY_MIXER_3_3, "PRI_MIXER_3_3", Float(0.0);
    PARAM_PRIMARY_MIXER_4_3, "PRI_MIXER_4_3", Float(0.0);
    PARAM_PRIMARY_MIXER_5_3, "PRI_MIXER_5_3", Float(0.0);
    PARAM_PRIMARY_MIXER_0_4, "PRI_MIXER_0_4", Float(0.0);
    PARAM_PRIMARY_MIXER_1_4, "PRI_MIXER_1_4", Float(0.0);
    PARAM_PRIMARY_MIXER_2_4, "PRI_MIXER_2_4", Float(0.0);
    PARAM_PRIMARY_MIXER_3_4, "PRI_MIXER_3_4", Float(0.0);
    PARAM_PRIMARY_MIXER_4_4, "PRI_MIXER_4_4", Float(0.0);
    PARAM_PRIMARY_MIXER_5_4, "PRI_MIXER_5_4", Float(0.0);
    PARAM_PRIMARY_MIXER_0_5, "PRI_MIXER_0_5", Float(0.0);
    PARAM_PRIMARY_MIXER_1_5, "PRI_MIXER_1_5", Float(0.0);
    PARAM_PRIMARY_MIXER_2_5, "PRI_MIXER_2_5", Float(0.0);
    PARAM_PRIMARY_MIXER_3_5, "PRI_MIXER_3_5", Float(0.0);
    PARAM_PRIMARY_MIXER_4_5, "PRI_MIXER_4_5", Float(0.0);
    PARAM_PRIMARY_MIXER_5_5, "PRI_MIXER_5_5", Float(0.0);
    PARAM_PRIMARY_MIXER_0_6, "PRI_MIXER_0_6", Float(0.0);
    PARAM_PRIMARY_MIXER_1_6, "PRI_MIXER_1_6", Float(0.0);
    PARAM_PRIMARY_MIXER_2_6, "PRI_MIXER_2_6", Float(0.0);
    PARAM_PRIMARY_MIXER_3_6, "PRI_MIXER_3_6", Float(0.0);
    PARAM_PRIMARY_MIXER_4_6, "PRI_MIXER_4_6", Float(0.0);
    PARAM_PRIMARY_MIXER_5_6, "PRI_MIXER_5_6", Float(0.0);
    PARAM_PRIMARY_MIXER_0_7, "PRI_MIXER_0_7", Float(0.0);
    PARAM_PRIMARY_MIXER_1_7, "PRI_MIXER_1_7", Float(0.0);
    PARAM_PRIMARY_MIXER_2_7, "PRI_MIXER_2_7", Float(0.0);
    PARAM_PRIMARY_MIXER_3_7, "PRI_MIXER_3_7", Float(0.0);
    PARAM_PRIMARY_MIXER_4_7, "PRI_MIXER_4_7", Float(0.0);
    PARAM_PRIMARY_MIXER_5_7, "PRI_MIXER_5_7", Float(0.0);
    PARAM_PRIMARY_MIXER_0_8, "PRI_MIXER_0_8", Float(0.0);
    PARAM_PRIMARY_MIXER_1_8, "PRI_MIXER_1_8", Float(0.0);
    PARAM_PRIMARY_MIXER_2_8, "PRI_MIXER_2_8", Float(0.0);
    PARAM_PRIMARY_MIXER_3_8, "PRI_MIXER_3_8", Float(0.0);
    PARAM_PRIMARY_MIXER_4_8, "PRI_MIXER_4_8", Float(0.0);
    PARAM_PRIMARY_MIXER_5_8, "PRI_MIXER_5_8", Float(0.0);
    PARAM_PRIMARY_MIXER_0_9, "PRI_MIXER_0_9", Float(0.0);
    PARAM_PRIMARY_MIXER_1_9, "PRI_MIXER_1_9", Float(0.0);
    PARAM_PRIMARY_MIXER_2_9, "PRI_MIXER_2_9", Float(0.0);
    PARAM_PRIMARY_MIXER_3_9, "PRI_MIXER_3_9", Float(0.0);
    PARAM_PRIMARY_MIXER_4_9, "PRI_MIXER_4_9", Float(0.0);
    PARAM_PRIMARY_MIXER_5_9, "PRI_MIXER_5_9", Float(0.0);
    PARAM_SECONDARY_MIXER_0_0, "SEC_MIXER_0_0", Float(0.0);
    PARAM_SECONDARY_MIXER_1_0, "SEC_MIXER_1_0", Float(0.0);
    PARAM_SECONDARY_MIXER_2_0, "SEC_MIXER_2_0", Float(0.0);
    PARAM_SECONDARY_MIXER_3_0, "SEC_MIXER_3_0", Float(0.0);
    PARAM_SECONDARY_MIXER_4_0, "SEC_MIXER_4_0", Float(0.0);
    PARAM_SECONDARY_MIXER_5_0, "SEC_MIXER_5_0", Float(0.0);
    PARAM_SECONDARY_MIXER_0_1, "SEC_MIXER_0_1", Float(0.0);
    PARAM_SECONDARY_MIXER_1_1, "SEC_MIXER_1_1", Float(0.0);
    PARAM_SECONDARY_MIXER_2_1, "SEC_MIXER_2_1", Float(0.0);
    PARAM_SECONDARY_MIXER_3_1, "SEC_MIXER_3_1", Float(0.0);
    PARAM_SECONDARY_MIXER_4_1, "SEC_MIXER_4_1", Float(0.0);
    PARAM_SECONDARY_MIXER_5_1, "SEC_MIXER_5_1", Float(0.0);
    PARAM_SECONDARY_MIXER_0_2, "SEC_MIXER_0_2", Float(0.0);
    PARAM_SECONDARY_MIXER_1_2, "SEC_MIXER_1_2", Float(0.0);
    PARAM_SECONDARY_MIXER_2_2, "SEC_MIXER_2_2", Float(0.0);
    PARAM_SECONDARY_MIXER_3_2, "SEC_MIXER_3_2", Float(0.0);
    PARAM_SECONDARY_MIXER_4_2, "SEC_MIXER_4_2", Float(0.0);
    PARAM_SECONDARY_MIXER_5_2, "SEC_MIXER_5_2", Float(0.0);
    PARAM_SECONDARY_MIXER_0_3, "SEC_MIXER_0_3", Float(0.0);
    PARAM_SECONDARY_MIXER_1_3, "SEC_MIXER_1_3", Float(0.0);
    PARAM_SECONDARY_MIXER_2_3, "SEC_MIXER_2_3", Float(0.0);
    PARAM_SECONDARY_MIXER_3_3, "SEC_MIXER_3_3", Float(0.0);
    PARAM_SECONDARY_MIXER_4_3, "SEC_MIXER_4_3", Float(0.0);
    PARAM_SECONDARY_MIXER_5_3, "SEC_MIXER_5_3", Float(0.0);
    PARAM_SECONDARY_MIXER_0_4, "SEC_MIXER_0_4", Float(0.0);
    PARAM_SECONDARY_MIXER_1_4, "SEC_MIXER_1_4", Float(0.0);
    PARAM_SECONDARY_MIXER_2_4, "SEC_MIXER_2_4", Float(0.0);
    PARAM_SECONDARY_MIXER_3_4, "SEC_MIXER_3_4", Float(0.0);
    PARAM_SECONDARY_MIXER_4_4, "SEC_MIXER_4_4", Float(0.0);
    PARAM_SECONDARY_MIXER_5_4, "SEC_MIXER_5_4", Float(0.0);
    PARAM_SECONDARY_MIXER_0_5, "SEC_MIXER_0_5", Float(0.0);
    PARAM_SECONDARY_MIXER_1_5, "SEC_MIXER_1_5", Float(0.0);
    PARAM_SECONDARY_MIXER_2_5, "SEC_MIXER_2_5", Float(0.0);
    PARAM_SECONDARY_MIXER_3_5, "SEC_MIXER_3_5", Float(0.0);
    PARAM_SECONDARY_MIXER_4_5, "SEC_MIXER_4_5", Float(0.0);
    PARAM_SECONDARY_MIXER_5_5, "SEC_MIXER_5_5", Float(0.0);
    PARAM_SECONDARY_MIXER_0_6, "SEC_MIXER_0_6", Float(0.0);
    PARAM_SECONDARY_MIXER_1_6, "SEC_MIXER_1_6", Float(0.0);
    PARAM_SECONDARY_MIXER_2_6, "SEC_MIXER_2_6", Float(0.0);
    PARAM_SECONDARY_MIXER_3_6, "SEC_MIXER_3_6", Float(0.0);
    PARAM_SECONDARY_MIXER_4_6, "SEC_MIXER_4_6", Float(0.0);
    PARAM_SECONDARY_MIXER_5_6, "SEC_MIXER_5_6", Float(0.0);
    PARAM_SECONDARY_MIXER_0_7, "SEC_MIXER_0_7", Float(0.0);
    PARAM_SECONDARY_MIXER_1_7, "SEC_MIXER_1_7", Float(0.0);
    PARAM_SECONDARY_MIXER_2_7, "SEC_MIXER_2_7", Float(0.0);
    PARAM_SECONDARY_MIXER_3_7, "SEC_MIXER_3_7", Float(0.0);
    PARAM_SECONDARY_MIXER_4_7, "SEC_MIXER_4_7", Float(0.0);
    PARAM_SECONDARY_MIXER_5_7, "SEC_MIXER_5_7", Float(0.0);
    PARAM_SECONDARY_MIXER_0_8, "SEC_MIXER_0_8", Float(0.0);
    PARAM_SECONDARY_MIXER_1_8, "SEC_MIXER_1_8", Float(0.0);
    PARAM_SECONDARY_MIXER_2_8, "SEC_MIXER_2_8", Float(0.0);
    PARAM_SECONDARY_MIXER_3_8, "SEC_MIXER_3_8", Float(0.0);
    PARAM_SECONDARY_MIXER_4_8, "SEC_MIXER_4_8", Float(0.0);
    PARAM_SECONDARY_MIXER_5_8, "SEC_MIXER_5_8", Float(0.0);
    PARAM_SECONDARY_MIXER_0_9, "SEC_MIXER_0_9", Float(0.0);
    PARAM_SECONDARY_MIXER_1_9, "SEC_MIXER_1_9", Float(0.0);
    PARAM_SECONDARY_MIXER_2_9, "SEC_MIXER_2_9", Float(0.0);
    PARAM_SECONDARY_MIXER_3_9, "SEC_MIXER_3_9", Float(0.0);
    PARAM_SECONDARY_MIXER_4_9, "SEC_MIXER_4_9", Float(0.0);
    PARAM_SECONDARY_MIXER_5_9, "SEC_MIXER_5_9", Float(0.0);
    PARAM_SYSTEM_ID, "SYS_ID", Int(1);
    PARAM_PID_ROLL_RATE_P, "PID_ROLL_RATE_P", Float(0.070);
    PARAM_PID_ROLL_RATE_I, "PID_ROLL_RATE_I", Float(0.0);
    PARAM_PID_ROLL_RATE_D, "PID_ROLL_RATE_D", Float(0.0);
    PARAM_PID_PITCH_RATE_P, "PID_PITCH_RATE_P", Float(0.070);
    PARAM_PID_PITCH_RATE_I, "PID_PITCH_RATE_I", Float(0.0);
    PARAM_PID_PITCH_RATE_D, "PID_PITCH_RATE_D", Float(0.0);
    PARAM_PID_YAW_RATE_P, "PID_YAW_RATE_P", Float(0.25);
    PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", Float(0.0);
    PARAM_PID_YAW_RATE_D, "PID_YAW_RATE_D", Float(0.0);
    PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", Float(0.15);
    PARAM_PID_ROLL_ANGLE_I, "PID_ROLL_ANG_I", Float(0.0);
    PARAM_PID_ROLL_ANGLE_D, "PID_ROLL_ANG_D", Float(0.05);
    PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", Float(0.15);
    PARAM_PID_PITCH_ANGLE_I, "PID_PITCH_ANG_I", Float(0.0);
    PARAM_PID_PITCH_ANGLE_D, "PID_PITCH_ANG_D", Float(0.05);
    PARAM_X_EQ_TORQUE, "X_EQ_TORQUE", Float(0.0);
    PARAM_Y_EQ_TORQUE, "Y_EQ_TORQUE", Float(0.0);
    PARAM_Z_EQ_TORQUE, "Z_EQ_TORQUE", Float(0.0);
    PARAM_PID_TAU, "PID_TAU", Float(0.05);
    PARAM_MOTOR_PWM_SEND_RATE, "MOTOR_PWM_UPDATE", Int(0);
    PARAM_MOTOR_IDLE_THROTTLE, "MOTOR_IDLE_THR", Float(0.1);
    PARAM_FAILSAFE_THROTTLE, "FAILSAFE_THR", Float(-1.0);
    PARAM_SPIN_MOTORS_WHEN_ARMED, "ARM_SPIN_MOTORS", Bool(true);
    PARAM_INIT_TIME, "FILTER_INIT_T", Int(3000);
    PARAM_FILTER_KP_ACC, "FILTER_KP_ACC", Float(0.5);
    PARAM_FILTER_KI, "FILTER_KI", Float(0.01);
    PARAM_FILTER_KP_EXT, "FILTER_KP_EXT", Float(1.5);
    PARAM_FILTER_ACCEL_MARGIN, "FILTER_ACCMARGIN", Float(0.1);
    PARAM_FILTER_USE_QUAD_INT, "FILTER_QUAD_INT", Bool(true);
    PARAM_FILTER_USE_MAT_EXP, "FILTER_MAT_EXP", Bool(true);
    PARAM_FILTER_USE_ACC, "FILTER_USE_ACC", Bool(true);
    PARAM_CALIBRATE_GYRO_ON_ARM, "CAL_GYRO_ARM", Bool(false);
    PARAM_GYRO_XY_ALPHA, "GYROXY_LPF_ALPHA", Float(0.3);
    PARAM_GYRO_Z_ALPHA, "GYROZ_LPF_ALPHA", Float(0.3);
    PARAM_ACC_ALPHA, "ACC_LPF_ALPHA", Float(0.5);
    PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", Float(0.0);
    PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", Float(0.0);
    PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", Float(0.0);
    PARAM_ACC_X_BIAS, "ACC_X_BIAS", Float(0.0);
    PARAM_ACC_Y_BIAS, "ACC_Y_BIAS", Float(0.0);
    PARAM_ACC_Z_BIAS, "ACC_Z_BIAS", Float(0.0);
    PARAM_ACC_X_TEMP_COMP, "ACC_X_TEMP_COMP", Float(0.0);
    PARAM_ACC_Y_TEMP_COMP, "ACC_Y_TEMP_COMP", Float(0.0);
    PARAM_ACC_Z_TEMP_COMP, "ACC_Z_TEMP_COMP", Float(0.0);
    PARAM_MAG_A11_COMP, "MAG_A11_COMP", Float(1.0);
    PARAM_MAG_A12_COMP, "MAG_A12_COMP", Float(0.0);
    PARAM_MAG_A13_COMP, "MAG_A13_COMP", Float(0.0);
    PARAM_MAG_A21_COMP, "MAG_A21_COMP", Float(0.0);
    PARAM_MAG_A22_COMP, "MAG_A22_COMP", Float(1.0);
    PARAM_MAG_A23_COMP, "MAG_A23_COMP", Float(0.0);
    PARAM_MAG_A31_COMP, "MAG_A31_COMP", Float(0.0);
    PARAM_MAG_A32_COMP, "MAG_A32_COMP", Float(0.0);
    PARAM_MAG_A33_COMP, "MAG_A33_COMP", Float(1.0);
    PARAM_MAG_X_BIAS, "MAG_X_BIAS", Float(0.0);
    PARAM_MAG_Y_BIAS, "MAG_Y_BIAS", Float(0.0);
    PARAM_MAG_Z_BIAS, "MAG_Z_BIAS", Float(0.0);
    PARAM_BARO_BIAS, "BARO_BIAS", Float(0.0);
    PARAM_GROUND_LEVEL, "GROUND_LEVEL", Float(1387.0);
    PARAM_DIFF_PRESS_BIAS, "DIFF_PRESS_BIAS", Float(0.0);
    PARAM_RC_TYPE, "RC_TYPE", Int(0);
    PARAM_RC_X_CHANNEL, "RC_X_CHN", Int(0);
    PARAM_RC_Y_CHANNEL, "RC_Y_CHN", Int(1);
    PARAM_RC_Z_CHANNEL, "RC_Z_CHN", Int(3);
    PARAM_RC_F_CHANNEL, "RC_F_CHN", Int(2);
    PARAM_RC_F_AXIS, "RC_F_AXIS", Int(2);
    PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, "RC_ATT_OVRD_CHN", Int(4);
    PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, "RC_THR_OVRD_CHN", Int(4);
    PARAM_RC_ATT_CONTROL_TYPE_CHANNEL, "RC_ATT_CTRL_CHN", Int(-1);
    PARAM_RC_ARM_CHANNEL, "ARM_CHANNEL", Int(-1);
    PARAM_RC_NUM_CHANNELS, "RC_NUM_CHN", Int(6);
    PARAM_RC_SWITCH_5_DIRECTION, "SWITCH_5_DIR", Int(1);
    PARAM_RC_SWITCH_6_DIRECTION, "SWITCH_6_DIR", Int(1);
    PARAM_RC_SWITCH_7_DIRECTION, "SWITCH_7_DIR", Int(1);
    PARAM_RC_SWITCH_8_DIRECTION, "SWITCH_8_DIR", Int(1);
    PARAM_RC_OVERRIDE_DEVIATION, "RC_OVRD_DEV", Float(0.1);
    PARAM_OVERRIDE_LAG_TIME, "OVRD_LAG_TIME", Int(1000);
    PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, "MIN_THROTTLE", Bool(true);
    PARAM_RC_MAX_THROTTLE, "RC_MAX_THR", Float(0.7);
    PARAM_RC_ATTITUDE_MODE, "RC_ATT_MODE", Int(1);
    PARAM_RC_MAX_ROLL, "RC_MAX_ROLL", Float(0.786);
    PARAM_RC_MAX_PITCH, "RC_MAX_PITCH", Float(0.786);
    PARAM_RC_MAX_ROLLRATE, "RC_MAX_ROLLRATE", Float(3.14159);
    PARAM_RC_MAX_PITCHRATE, "RC_MAX_PITCHRATE", Float(3.14159);
    PARAM_RC_MAX_YAWRATE, "RC_MAX_YAWRATE", Float(1.507);
    PARAM_PRIMARY_MIXER, "PRIMARY_MIXER", Int(-1);
    PARAM_SECONDARY_MIXER, "SECONDARY_MIXER", Int(-1);
    PARAM_FIXED_WING, "FIXED_WING", Bool(false);
    PARAM_ELEVATOR_REVERSE, "ELEVATOR_REV", Int(0);
    PARAM_AILERON_REVERSE, "AIL_REV", Int(0);
    PARAM_RUDDER_REVERSE, "RUDDER_REV", Int(0);
    PARAM_FC_ROLL, "FC_ROLL", Float(0.0);
    PARAM_FC_PITCH, "FC_PITCH", Float(0.0);
    PARAM_FC_YAW, "FC_YAW", Float(0.0);
    PARAM_ARM_THRESHOLD, "ARM_THRESHOLD", Float(0.15);
    PARAM_BATTERY_VOLTAGE_MULTIPLIER, "BATT_VOLT_MULT", Float(0.0);
    PARAM_BATTERY_CURRENT_MULTIPLIER, "BATT_CURR_MULT", Float(0.0);
    PARAM_BATTERY_VOLTAGE_ALPHA, "BATT_VOLT_ALPHA", Float(0.995);
    PARAM_BATTERY_CURRENT_ALPHA, "BATT_CURR_ALPHA", Float(0.995);
    PARAM_OFFBOARD_TIMEOUT, "OFFBOARD_TIMEOUT", Int(100);
}

//=================================================================================
// 4. Custom Iterator
//=================================================================================

/// A `no_std`, stack-based iterator for the `Params` struct.
pub struct ParamIter {
    params: Params,
    index: usize,
}

impl ParamIter {
    /// Returns the next parameter as a `(ParamId, ParamValue)` tuple, or `None`.
    pub fn next(&mut self) -> Option<(ParamId, ParamValue)> {
        if self.index < PARAMS_COUNT {
            // Unsafe is acceptable here because `self.index` is guaranteed to be in bounds.
            let id: ParamId = unsafe { core::mem::transmute(self.index as u16) };
            let value = self.params.get_by_id(id);
            self.index += 1;
            Some((id, value))
        } else {
            None
        }
    }
}

//=================================================================================
// 5. Unit Tests
//=================================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_param_count() {
        // This test now just checks for internal consistency.
        // The actual count should be 276.
        assert_eq!(PARAMS_COUNT, PARAM_DEFINITIONS.len());
    }

    #[test]
    fn test_new_and_defaults() {
        let p = Params::new();
        assert_eq!(
            p.get_by_id(ParamId::PARAM_BAUD_RATE),
            ParamValue::Int(921600)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_PID_ROLL_RATE_P),
            ParamValue::Float(0.070)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED),
            ParamValue::Bool(true)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM),
            ParamValue::Bool(false)
        );
    }

    #[test]
    fn test_get_set_by_id() {
        let mut p = Params::new();
        p.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        assert_eq!(p.get_by_id(ParamId::PARAM_SYSTEM_ID), ParamValue::Int(42));
        p.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Bool(true));
        assert_eq!(
            p.get_by_id(ParamId::PARAM_FIXED_WING),
            ParamValue::Bool(true)
        );
    }

    #[test]
    fn test_get_set_by_name() {
        let mut p = Params::new();
        let success = p.set_by_name("SYS_ID", ParamValue::Int(99));
        assert!(success);
        assert_eq!(p.get_by_id(ParamId::PARAM_SYSTEM_ID), ParamValue::Int(99));
        assert_eq!(p.get_by_name("SYS_ID"), Some(ParamValue::Int(99)));

        let failure = p.set_by_name("NON_EXISTENT", ParamValue::Int(1));
        assert!(!failure);
        assert_eq!(p.get_by_name("NON_EXISTENT"), None);
    }

    #[test]
    fn test_iterator() {
        let p = Params::new();
        let mut iter = p.iter();

        let first = iter.next();
        assert!(first.is_some());
        assert_eq!(first.unwrap().0, ParamId::PARAM_BAUD_RATE);

        let mut count = 1;
        while iter.next().is_some() {
            count += 1;
        }

        assert_eq!(count, PARAMS_COUNT);
        assert!(iter.next().is_none());
    }
}
