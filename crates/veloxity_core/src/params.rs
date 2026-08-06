#![allow(non_camel_case_types)]

pub mod reactions;
pub mod service;
pub mod storage;

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

            /// Returns a parameter ID from its table index.
            pub fn from_index(index: usize) -> Option<Self> {
                PARAM_DEFINITIONS.get(index).map(|definition| definition.id)
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
    PARAM_VOLT_MAX, "BATT_VOLT_MAX", Float(25.0);
    PARAM_USE_MOTOR_PARAMETERS, "USE_MOTOR_PARAM", Int(0);
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
    PARAM_PRIMARY_MIXER_6_0, "PRI_MIXER_6_0", Float(0.0);
    PARAM_PRIMARY_MIXER_7_0, "PRI_MIXER_7_0", Float(0.0);
    PARAM_PRIMARY_MIXER_8_0, "PRI_MIXER_8_0", Float(0.0);
    PARAM_PRIMARY_MIXER_9_0, "PRI_MIXER_9_0", Float(0.0);
    PARAM_PRIMARY_MIXER_0_1, "PRI_MIXER_0_1", Float(0.0);
    PARAM_PRIMARY_MIXER_1_1, "PRI_MIXER_1_1", Float(0.0);
    PARAM_PRIMARY_MIXER_2_1, "PRI_MIXER_2_1", Float(0.0);
    PARAM_PRIMARY_MIXER_3_1, "PRI_MIXER_3_1", Float(0.0);
    PARAM_PRIMARY_MIXER_4_1, "PRI_MIXER_4_1", Float(0.0);
    PARAM_PRIMARY_MIXER_5_1, "PRI_MIXER_5_1", Float(0.0);
    PARAM_PRIMARY_MIXER_6_1, "PRI_MIXER_6_1", Float(0.0);
    PARAM_PRIMARY_MIXER_7_1, "PRI_MIXER_7_1", Float(0.0);
    PARAM_PRIMARY_MIXER_8_1, "PRI_MIXER_8_1", Float(0.0);
    PARAM_PRIMARY_MIXER_9_1, "PRI_MIXER_9_1", Float(0.0);
    PARAM_PRIMARY_MIXER_0_2, "PRI_MIXER_0_2", Float(0.0);
    PARAM_PRIMARY_MIXER_1_2, "PRI_MIXER_1_2", Float(0.0);
    PARAM_PRIMARY_MIXER_2_2, "PRI_MIXER_2_2", Float(0.0);
    PARAM_PRIMARY_MIXER_3_2, "PRI_MIXER_3_2", Float(0.0);
    PARAM_PRIMARY_MIXER_4_2, "PRI_MIXER_4_2", Float(0.0);
    PARAM_PRIMARY_MIXER_5_2, "PRI_MIXER_5_2", Float(0.0);
    PARAM_PRIMARY_MIXER_6_2, "PRI_MIXER_6_2", Float(0.0);
    PARAM_PRIMARY_MIXER_7_2, "PRI_MIXER_7_2", Float(0.0);
    PARAM_PRIMARY_MIXER_8_2, "PRI_MIXER_8_2", Float(0.0);
    PARAM_PRIMARY_MIXER_9_2, "PRI_MIXER_9_2", Float(0.0);
    PARAM_PRIMARY_MIXER_0_3, "PRI_MIXER_0_3", Float(0.0);
    PARAM_PRIMARY_MIXER_1_3, "PRI_MIXER_1_3", Float(0.0);
    PARAM_PRIMARY_MIXER_2_3, "PRI_MIXER_2_3", Float(0.0);
    PARAM_PRIMARY_MIXER_3_3, "PRI_MIXER_3_3", Float(0.0);
    PARAM_PRIMARY_MIXER_4_3, "PRI_MIXER_4_3", Float(0.0);
    PARAM_PRIMARY_MIXER_5_3, "PRI_MIXER_5_3", Float(0.0);
    PARAM_PRIMARY_MIXER_6_3, "PRI_MIXER_6_3", Float(0.0);
    PARAM_PRIMARY_MIXER_7_3, "PRI_MIXER_7_3", Float(0.0);
    PARAM_PRIMARY_MIXER_8_3, "PRI_MIXER_8_3", Float(0.0);
    PARAM_PRIMARY_MIXER_9_3, "PRI_MIXER_9_3", Float(0.0);
    PARAM_PRIMARY_MIXER_0_4, "PRI_MIXER_0_4", Float(0.0);
    PARAM_PRIMARY_MIXER_1_4, "PRI_MIXER_1_4", Float(0.0);
    PARAM_PRIMARY_MIXER_2_4, "PRI_MIXER_2_4", Float(0.0);
    PARAM_PRIMARY_MIXER_3_4, "PRI_MIXER_3_4", Float(0.0);
    PARAM_PRIMARY_MIXER_4_4, "PRI_MIXER_4_4", Float(0.0);
    PARAM_PRIMARY_MIXER_5_4, "PRI_MIXER_5_4", Float(0.0);
    PARAM_PRIMARY_MIXER_6_4, "PRI_MIXER_6_4", Float(0.0);
    PARAM_PRIMARY_MIXER_7_4, "PRI_MIXER_7_4", Float(0.0);
    PARAM_PRIMARY_MIXER_8_4, "PRI_MIXER_8_4", Float(0.0);
    PARAM_PRIMARY_MIXER_9_4, "PRI_MIXER_9_4", Float(0.0);
    PARAM_PRIMARY_MIXER_0_5, "PRI_MIXER_0_5", Float(0.0);
    PARAM_PRIMARY_MIXER_1_5, "PRI_MIXER_1_5", Float(0.0);
    PARAM_PRIMARY_MIXER_2_5, "PRI_MIXER_2_5", Float(0.0);
    PARAM_PRIMARY_MIXER_3_5, "PRI_MIXER_3_5", Float(0.0);
    PARAM_PRIMARY_MIXER_4_5, "PRI_MIXER_4_5", Float(0.0);
    PARAM_PRIMARY_MIXER_5_5, "PRI_MIXER_5_5", Float(0.0);
    PARAM_PRIMARY_MIXER_6_5, "PRI_MIXER_6_5", Float(0.0);
    PARAM_PRIMARY_MIXER_7_5, "PRI_MIXER_7_5", Float(0.0);
    PARAM_PRIMARY_MIXER_8_5, "PRI_MIXER_8_5", Float(0.0);
    PARAM_PRIMARY_MIXER_9_5, "PRI_MIXER_9_5", Float(0.0);
    PARAM_PRIMARY_MIXER_0_6, "PRI_MIXER_0_6", Float(0.0);
    PARAM_PRIMARY_MIXER_1_6, "PRI_MIXER_1_6", Float(0.0);
    PARAM_PRIMARY_MIXER_2_6, "PRI_MIXER_2_6", Float(0.0);
    PARAM_PRIMARY_MIXER_3_6, "PRI_MIXER_3_6", Float(0.0);
    PARAM_PRIMARY_MIXER_4_6, "PRI_MIXER_4_6", Float(0.0);
    PARAM_PRIMARY_MIXER_5_6, "PRI_MIXER_5_6", Float(0.0);
    PARAM_PRIMARY_MIXER_6_6, "PRI_MIXER_6_6", Float(0.0);
    PARAM_PRIMARY_MIXER_7_6, "PRI_MIXER_7_6", Float(0.0);
    PARAM_PRIMARY_MIXER_8_6, "PRI_MIXER_8_6", Float(0.0);
    PARAM_PRIMARY_MIXER_9_6, "PRI_MIXER_9_6", Float(0.0);
    PARAM_PRIMARY_MIXER_0_7, "PRI_MIXER_0_7", Float(0.0);
    PARAM_PRIMARY_MIXER_1_7, "PRI_MIXER_1_7", Float(0.0);
    PARAM_PRIMARY_MIXER_2_7, "PRI_MIXER_2_7", Float(0.0);
    PARAM_PRIMARY_MIXER_3_7, "PRI_MIXER_3_7", Float(0.0);
    PARAM_PRIMARY_MIXER_4_7, "PRI_MIXER_4_7", Float(0.0);
    PARAM_PRIMARY_MIXER_5_7, "PRI_MIXER_5_7", Float(0.0);
    PARAM_PRIMARY_MIXER_6_7, "PRI_MIXER_6_7", Float(0.0);
    PARAM_PRIMARY_MIXER_7_7, "PRI_MIXER_7_7", Float(0.0);
    PARAM_PRIMARY_MIXER_8_7, "PRI_MIXER_8_7", Float(0.0);
    PARAM_PRIMARY_MIXER_9_7, "PRI_MIXER_9_7", Float(0.0);
    PARAM_PRIMARY_MIXER_0_8, "PRI_MIXER_0_8", Float(0.0);
    PARAM_PRIMARY_MIXER_1_8, "PRI_MIXER_1_8", Float(0.0);
    PARAM_PRIMARY_MIXER_2_8, "PRI_MIXER_2_8", Float(0.0);
    PARAM_PRIMARY_MIXER_3_8, "PRI_MIXER_3_8", Float(0.0);
    PARAM_PRIMARY_MIXER_4_8, "PRI_MIXER_4_8", Float(0.0);
    PARAM_PRIMARY_MIXER_5_8, "PRI_MIXER_5_8", Float(0.0);
    PARAM_PRIMARY_MIXER_6_8, "PRI_MIXER_6_8", Float(0.0);
    PARAM_PRIMARY_MIXER_7_8, "PRI_MIXER_7_8", Float(0.0);
    PARAM_PRIMARY_MIXER_8_8, "PRI_MIXER_8_8", Float(0.0);
    PARAM_PRIMARY_MIXER_9_8, "PRI_MIXER_9_8", Float(0.0);
    PARAM_PRIMARY_MIXER_0_9, "PRI_MIXER_0_9", Float(0.0);
    PARAM_PRIMARY_MIXER_1_9, "PRI_MIXER_1_9", Float(0.0);
    PARAM_PRIMARY_MIXER_2_9, "PRI_MIXER_2_9", Float(0.0);
    PARAM_PRIMARY_MIXER_3_9, "PRI_MIXER_3_9", Float(0.0);
    PARAM_PRIMARY_MIXER_4_9, "PRI_MIXER_4_9", Float(0.0);
    PARAM_PRIMARY_MIXER_5_9, "PRI_MIXER_5_9", Float(0.0);
    PARAM_PRIMARY_MIXER_6_9, "PRI_MIXER_6_9", Float(0.0);
    PARAM_PRIMARY_MIXER_7_9, "PRI_MIXER_7_9", Float(0.0);
    PARAM_PRIMARY_MIXER_8_9, "PRI_MIXER_8_9", Float(0.0);
    PARAM_PRIMARY_MIXER_9_9, "PRI_MIXER_9_9", Float(0.0);
    PARAM_SECONDARY_MIXER_0_0, "SEC_MIXER_0_0", Float(0.0);
    PARAM_SECONDARY_MIXER_1_0, "SEC_MIXER_1_0", Float(0.0);
    PARAM_SECONDARY_MIXER_2_0, "SEC_MIXER_2_0", Float(0.0);
    PARAM_SECONDARY_MIXER_3_0, "SEC_MIXER_3_0", Float(0.0);
    PARAM_SECONDARY_MIXER_4_0, "SEC_MIXER_4_0", Float(0.0);
    PARAM_SECONDARY_MIXER_5_0, "SEC_MIXER_5_0", Float(0.0);
    PARAM_SECONDARY_MIXER_6_0, "SEC_MIXER_6_0", Float(0.0);
    PARAM_SECONDARY_MIXER_7_0, "SEC_MIXER_7_0", Float(0.0);
    PARAM_SECONDARY_MIXER_8_0, "SEC_MIXER_8_0", Float(0.0);
    PARAM_SECONDARY_MIXER_9_0, "SEC_MIXER_9_0", Float(0.0);
    PARAM_SECONDARY_MIXER_0_1, "SEC_MIXER_0_1", Float(0.0);
    PARAM_SECONDARY_MIXER_1_1, "SEC_MIXER_1_1", Float(0.0);
    PARAM_SECONDARY_MIXER_2_1, "SEC_MIXER_2_1", Float(0.0);
    PARAM_SECONDARY_MIXER_3_1, "SEC_MIXER_3_1", Float(0.0);
    PARAM_SECONDARY_MIXER_4_1, "SEC_MIXER_4_1", Float(0.0);
    PARAM_SECONDARY_MIXER_5_1, "SEC_MIXER_5_1", Float(0.0);
    PARAM_SECONDARY_MIXER_6_1, "SEC_MIXER_6_1", Float(0.0);
    PARAM_SECONDARY_MIXER_7_1, "SEC_MIXER_7_1", Float(0.0);
    PARAM_SECONDARY_MIXER_8_1, "SEC_MIXER_8_1", Float(0.0);
    PARAM_SECONDARY_MIXER_9_1, "SEC_MIXER_9_1", Float(0.0);
    PARAM_SECONDARY_MIXER_0_2, "SEC_MIXER_0_2", Float(0.0);
    PARAM_SECONDARY_MIXER_1_2, "SEC_MIXER_1_2", Float(0.0);
    PARAM_SECONDARY_MIXER_2_2, "SEC_MIXER_2_2", Float(0.0);
    PARAM_SECONDARY_MIXER_3_2, "SEC_MIXER_3_2", Float(0.0);
    PARAM_SECONDARY_MIXER_4_2, "SEC_MIXER_4_2", Float(0.0);
    PARAM_SECONDARY_MIXER_5_2, "SEC_MIXER_5_2", Float(0.0);
    PARAM_SECONDARY_MIXER_6_2, "SEC_MIXER_6_2", Float(0.0);
    PARAM_SECONDARY_MIXER_7_2, "SEC_MIXER_7_2", Float(0.0);
    PARAM_SECONDARY_MIXER_8_2, "SEC_MIXER_8_2", Float(0.0);
    PARAM_SECONDARY_MIXER_9_2, "SEC_MIXER_9_2", Float(0.0);
    PARAM_SECONDARY_MIXER_0_3, "SEC_MIXER_0_3", Float(0.0);
    PARAM_SECONDARY_MIXER_1_3, "SEC_MIXER_1_3", Float(0.0);
    PARAM_SECONDARY_MIXER_2_3, "SEC_MIXER_2_3", Float(0.0);
    PARAM_SECONDARY_MIXER_3_3, "SEC_MIXER_3_3", Float(0.0);
    PARAM_SECONDARY_MIXER_4_3, "SEC_MIXER_4_3", Float(0.0);
    PARAM_SECONDARY_MIXER_5_3, "SEC_MIXER_5_3", Float(0.0);
    PARAM_SECONDARY_MIXER_6_3, "SEC_MIXER_6_3", Float(0.0);
    PARAM_SECONDARY_MIXER_7_3, "SEC_MIXER_7_3", Float(0.0);
    PARAM_SECONDARY_MIXER_8_3, "SEC_MIXER_8_3", Float(0.0);
    PARAM_SECONDARY_MIXER_9_3, "SEC_MIXER_9_3", Float(0.0);
    PARAM_SECONDARY_MIXER_0_4, "SEC_MIXER_0_4", Float(0.0);
    PARAM_SECONDARY_MIXER_1_4, "SEC_MIXER_1_4", Float(0.0);
    PARAM_SECONDARY_MIXER_2_4, "SEC_MIXER_2_4", Float(0.0);
    PARAM_SECONDARY_MIXER_3_4, "SEC_MIXER_3_4", Float(0.0);
    PARAM_SECONDARY_MIXER_4_4, "SEC_MIXER_4_4", Float(0.0);
    PARAM_SECONDARY_MIXER_5_4, "SEC_MIXER_5_4", Float(0.0);
    PARAM_SECONDARY_MIXER_6_4, "SEC_MIXER_6_4", Float(0.0);
    PARAM_SECONDARY_MIXER_7_4, "SEC_MIXER_7_4", Float(0.0);
    PARAM_SECONDARY_MIXER_8_4, "SEC_MIXER_8_4", Float(0.0);
    PARAM_SECONDARY_MIXER_9_4, "SEC_MIXER_9_4", Float(0.0);
    PARAM_SECONDARY_MIXER_0_5, "SEC_MIXER_0_5", Float(0.0);
    PARAM_SECONDARY_MIXER_1_5, "SEC_MIXER_1_5", Float(0.0);
    PARAM_SECONDARY_MIXER_2_5, "SEC_MIXER_2_5", Float(0.0);
    PARAM_SECONDARY_MIXER_3_5, "SEC_MIXER_3_5", Float(0.0);
    PARAM_SECONDARY_MIXER_4_5, "SEC_MIXER_4_5", Float(0.0);
    PARAM_SECONDARY_MIXER_5_5, "SEC_MIXER_5_5", Float(0.0);
    PARAM_SECONDARY_MIXER_6_5, "SEC_MIXER_6_5", Float(0.0);
    PARAM_SECONDARY_MIXER_7_5, "SEC_MIXER_7_5", Float(0.0);
    PARAM_SECONDARY_MIXER_8_5, "SEC_MIXER_8_5", Float(0.0);
    PARAM_SECONDARY_MIXER_9_5, "SEC_MIXER_9_5", Float(0.0);
    PARAM_SECONDARY_MIXER_0_6, "SEC_MIXER_0_6", Float(0.0);
    PARAM_SECONDARY_MIXER_1_6, "SEC_MIXER_1_6", Float(0.0);
    PARAM_SECONDARY_MIXER_2_6, "SEC_MIXER_2_6", Float(0.0);
    PARAM_SECONDARY_MIXER_3_6, "SEC_MIXER_3_6", Float(0.0);
    PARAM_SECONDARY_MIXER_4_6, "SEC_MIXER_4_6", Float(0.0);
    PARAM_SECONDARY_MIXER_5_6, "SEC_MIXER_5_6", Float(0.0);
    PARAM_SECONDARY_MIXER_6_6, "SEC_MIXER_6_6", Float(0.0);
    PARAM_SECONDARY_MIXER_7_6, "SEC_MIXER_7_6", Float(0.0);
    PARAM_SECONDARY_MIXER_8_6, "SEC_MIXER_8_6", Float(0.0);
    PARAM_SECONDARY_MIXER_9_6, "SEC_MIXER_9_6", Float(0.0);
    PARAM_SECONDARY_MIXER_0_7, "SEC_MIXER_0_7", Float(0.0);
    PARAM_SECONDARY_MIXER_1_7, "SEC_MIXER_1_7", Float(0.0);
    PARAM_SECONDARY_MIXER_2_7, "SEC_MIXER_2_7", Float(0.0);
    PARAM_SECONDARY_MIXER_3_7, "SEC_MIXER_3_7", Float(0.0);
    PARAM_SECONDARY_MIXER_4_7, "SEC_MIXER_4_7", Float(0.0);
    PARAM_SECONDARY_MIXER_5_7, "SEC_MIXER_5_7", Float(0.0);
    PARAM_SECONDARY_MIXER_6_7, "SEC_MIXER_6_7", Float(0.0);
    PARAM_SECONDARY_MIXER_7_7, "SEC_MIXER_7_7", Float(0.0);
    PARAM_SECONDARY_MIXER_8_7, "SEC_MIXER_8_7", Float(0.0);
    PARAM_SECONDARY_MIXER_9_7, "SEC_MIXER_9_7", Float(0.0);
    PARAM_SECONDARY_MIXER_0_8, "SEC_MIXER_0_8", Float(0.0);
    PARAM_SECONDARY_MIXER_1_8, "SEC_MIXER_1_8", Float(0.0);
    PARAM_SECONDARY_MIXER_2_8, "SEC_MIXER_2_8", Float(0.0);
    PARAM_SECONDARY_MIXER_3_8, "SEC_MIXER_3_8", Float(0.0);
    PARAM_SECONDARY_MIXER_4_8, "SEC_MIXER_4_8", Float(0.0);
    PARAM_SECONDARY_MIXER_5_8, "SEC_MIXER_5_8", Float(0.0);
    PARAM_SECONDARY_MIXER_6_8, "SEC_MIXER_6_8", Float(0.0);
    PARAM_SECONDARY_MIXER_7_8, "SEC_MIXER_7_8", Float(0.0);
    PARAM_SECONDARY_MIXER_8_8, "SEC_MIXER_8_8", Float(0.0);
    PARAM_SECONDARY_MIXER_9_8, "SEC_MIXER_9_8", Float(0.0);
    PARAM_SECONDARY_MIXER_0_9, "SEC_MIXER_0_9", Float(0.0);
    PARAM_SECONDARY_MIXER_1_9, "SEC_MIXER_1_9", Float(0.0);
    PARAM_SECONDARY_MIXER_2_9, "SEC_MIXER_2_9", Float(0.0);
    PARAM_SECONDARY_MIXER_3_9, "SEC_MIXER_3_9", Float(0.0);
    PARAM_SECONDARY_MIXER_4_9, "SEC_MIXER_4_9", Float(0.0);
    PARAM_SECONDARY_MIXER_5_9, "SEC_MIXER_5_9", Float(0.0);
    PARAM_SECONDARY_MIXER_6_9, "SEC_MIXER_6_9", Float(0.0);
    PARAM_SECONDARY_MIXER_7_9, "SEC_MIXER_7_9", Float(0.0);
    PARAM_SECONDARY_MIXER_8_9, "SEC_MIXER_8_9", Float(0.0);
    PARAM_SECONDARY_MIXER_9_9, "SEC_MIXER_9_9", Float(0.0);
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
    PARAM_MOTOR_IDLE_THROTTLE, "MOTOR_IDLE_THR", Float(0.1);
    PARAM_FAILSAFE_THROTTLE, "FAILSAFE_THR", Float(-1.0);
    PARAM_SPIN_MOTORS_WHEN_ARMED, "ARM_SPIN_MOTORS", Int(1);
    // Physical output-channel allow mask. Bit N controls channel N regardless
    // of whether the mixer labels it as a motor, servo, GPIO, or auxiliary
    // output. Zero is also a global hardware-output inhibit.
    PARAM_CHANNEL_OUTPUT_MASK, "CHN_OUTPUT_MASK", Int(0x0f);
    PARAM_INIT_TIME, "FILT_INIT_T", Int(3000);
    PARAM_FILTER_KP_ACC, "FILT_ACC_KP", Float(0.5);
    PARAM_FILTER_KI, "FILT_KI", Float(0.01);
    PARAM_FILTER_KP_EXT, "FILT_EXT_KP", Float(1.5);
    PARAM_FILTER_ACCEL_MARGIN, "FILT_ACC_MARGIN", Float(0.1);
    PARAM_FILTER_USE_QUAD_INT, "FILT_QUAD_INT", Int(1);
    PARAM_FILTER_USE_MAT_EXP, "FILT_MAT_EXP", Int(1);
    PARAM_FILTER_USE_ACC, "FILT_USE_ACC", Int(1);
    PARAM_CALIBRATE_GYRO_ON_ARM, "ARM_CAL_GYRO", Int(0);
    PARAM_GYRO_XY_ALPHA, "GYRO_XY_LPF", Float(0.3);
    PARAM_GYRO_Z_ALPHA, "GYRO_Z_LPF", Float(0.3);
    PARAM_ACC_ALPHA, "ACC_LPF", Float(0.5);
    PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", Float(0.0);
    PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", Float(0.0);
    PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", Float(0.0);
    PARAM_ACC_X_BIAS, "ACC_X_BIAS", Float(0.0);
    PARAM_ACC_Y_BIAS, "ACC_Y_BIAS", Float(0.0);
    PARAM_ACC_Z_BIAS, "ACC_Z_BIAS", Float(0.0);
    PARAM_ACC_X_TEMP_COMP, "ACC_X_TEMP_COMP", Float(0.0);
    PARAM_ACC_Y_TEMP_COMP, "ACC_Y_TEMP_COMP", Float(0.0);
    PARAM_ACC_Z_TEMP_COMP, "ACC_Z_TEMP_COMP", Float(0.0);
    PARAM_MAG_A00_COMP, "MAG_CAL_A00", Float(1.0);
    PARAM_MAG_A01_COMP, "MAG_CAL_A01", Float(0.0);
    PARAM_MAG_A02_COMP, "MAG_CAL_A02", Float(0.0);
    PARAM_MAG_A10_COMP, "MAG_CAL_A10", Float(0.0);
    PARAM_MAG_A11_COMP, "MAG_CAL_A11", Float(1.0);
    PARAM_MAG_A12_COMP, "MAG_CAL_A12", Float(0.0);
    PARAM_MAG_A20_COMP, "MAG_CAL_A20", Float(0.0);
    PARAM_MAG_A21_COMP, "MAG_CAL_A21", Float(0.0);
    PARAM_MAG_A22_COMP, "MAG_CAL_A22", Float(1.0);
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
    PARAM_RC_ARM_CHANNEL, "RC_ARM_CHN", Int(-1);
    PARAM_RC_NUM_CHANNELS, "RC_NUM_CHN", Int(6);
    PARAM_RC_SWITCH_5_DIRECTION, "SWITCH_5_DIR", Int(1);
    PARAM_RC_SWITCH_6_DIRECTION, "SWITCH_6_DIR", Int(1);
    PARAM_RC_SWITCH_7_DIRECTION, "SWITCH_7_DIR", Int(1);
    PARAM_RC_SWITCH_8_DIRECTION, "SWITCH_8_DIR", Int(1);
    PARAM_RC_OVERRIDE_DEVIATION, "RC_OVRD_DEV", Float(0.1);
    PARAM_OVERRIDE_LAG_TIME, "RC_OVRD_LAG_T", Int(1000);
    PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, "TAKE_MIN_THR", Int(1);
    PARAM_RC_MAX_THROTTLE, "RC_MAX_THR", Float(0.7);
    PARAM_RC_ATTITUDE_MODE, "RC_ATT_MODE", Int(1);
    PARAM_ALLOW_UNHEALTHY_ESTIMATOR, "BYPASS_UNH_EST", Int(1);
    PARAM_EST_ANGLE_LOCKOUT, "EST_ANG_LOCKOUT", Int(0);
    PARAM_RC_MAX_ROLL, "RC_MAX_ROLL", Float(0.786);
    PARAM_RC_MAX_PITCH, "RC_MAX_PITCH", Float(0.786);
    PARAM_RC_MAX_ROLLRATE, "RC_MAX_ROLLRATE", Float(3.14159);
    PARAM_RC_MAX_PITCHRATE, "RC_MAX_PITCHRATE", Float(3.14159);
    PARAM_RC_MAX_YAWRATE, "RC_MAX_YAWRATE", Float(1.507);
    PARAM_PRIMARY_MIXER, "PRIMARY_MIXER", Int(-1);
    PARAM_SECONDARY_MIXER, "SECONDARY_MIXER", Int(-1);
    PARAM_FIXED_WING, "FIXED_WING", Int(0);
    PARAM_ELEVATOR_REVERSE, "REV_ELEVATOR", Int(0);
    PARAM_AILERON_REVERSE, "REV_AILERON", Int(0);
    PARAM_RUDDER_REVERSE, "REV_RUDDER", Int(0);
    PARAM_IMU_ROLL, "IMU_ROLL", Float(0.0);
    PARAM_IMU_PITCH, "IMU_PITCH", Float(0.0);
    PARAM_IMU_YAW, "IMU_YAW", Float(0.0);
    PARAM_MAG_ROLL, "MAG_ROLL", Float(0.0);
    PARAM_MAG_PITCH, "MAG_PITCH", Float(0.0);
    PARAM_MAG_YAW, "MAG_YAW", Float(0.0);
    PARAM_ARM_THRESHOLD, "RC_ARM_THRESHOLD", Float(0.15);
    PARAM_BATTERY_VOLTAGE_MULTIPLIER, "BATT_VOLT_MULT", Float(1.0);
    PARAM_BATTERY_CURRENT_MULTIPLIER, "BATT_CURR_MULT", Float(1.0);
    PARAM_BATTERY_VOLTAGE_ALPHA, "BATT_VOLT_LPF", Float(0.995);
    PARAM_BATTERY_CURRENT_ALPHA, "BATT_CURR_LPF", Float(0.995);
    PARAM_OFFBOARD_TIMEOUT, "OFFBOARD_TIMEOUT", Int(100);
    PARAM_RC_OUTPUT_KILL_CHANNEL, "RC_KILL_CHN", Int(-1);

    // Telemetry publication rates. -1 disables a stream, 0 publishes whenever
    // eligible, and positive values request a fixed rate in hertz.
    PARAM_TELEM_HEARTBEAT_HZ, "TEL_HB_HZ", Int(1);
    PARAM_TELEM_STATUS_HZ, "TEL_STATUS_HZ", Int(10);
    PARAM_TELEM_IMU_HZ, "TEL_IMU_HZ", Int(0);
    PARAM_TELEM_ATTITUDE_HZ, "TEL_ATT_HZ", Int(0);
    PARAM_TELEM_OUTPUT_RAW_HZ, "TEL_OUT_HZ", Int(50);
    PARAM_TELEM_DIFF_PRESSURE_HZ, "TEL_DIFF_HZ", Int(0);
    PARAM_TELEM_BARO_HZ, "TEL_BARO_HZ", Int(0);
    PARAM_TELEM_MAG_HZ, "TEL_MAG_HZ", Int(0);
    PARAM_TELEM_RANGE_HZ, "TEL_RANGE_HZ", Int(0);
    PARAM_TELEM_BATTERY_HZ, "TEL_BATT_HZ", Int(0);
    PARAM_TELEM_GNSS_HZ, "TEL_GNSS_HZ", Int(0);
    PARAM_TELEM_RC_HZ, "TEL_RC_HZ", Int(0);
}

//=================================================================================
// 4. Unit Tests
//=================================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_param_count() {
        assert_eq!(PARAMS_COUNT, PARAM_DEFINITIONS.len());
        assert_eq!(PARAMS_COUNT, 349);
    }

    #[test]
    fn rosflight_2_0_param_surface_keeps_known_compatibility_names() {
        assert_eq!(ParamId::PARAM_VOLT_MAX.as_str(), "BATT_VOLT_MAX");
        assert_eq!(ParamId::PARAM_PRIMARY_MIXER_9_9.as_str(), "PRI_MIXER_9_9");
        assert_eq!(ParamId::PARAM_SECONDARY_MIXER_9_9.as_str(), "SEC_MIXER_9_9");
        assert_eq!(ParamId::PARAM_INIT_TIME.as_str(), "FILT_INIT_T");
        assert_eq!(ParamId::PARAM_FILTER_KP_ACC.as_str(), "FILT_ACC_KP");
        assert_eq!(ParamId::PARAM_MAG_A00_COMP.as_str(), "MAG_CAL_A00");
        assert_eq!(ParamId::PARAM_RC_ARM_CHANNEL.as_str(), "RC_ARM_CHN");
        assert_eq!(ParamId::PARAM_OVERRIDE_LAG_TIME.as_str(), "RC_OVRD_LAG_T");
        assert_eq!(
            ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE.as_str(),
            "TAKE_MIN_THR"
        );
        assert_eq!(
            ParamId::PARAM_ALLOW_UNHEALTHY_ESTIMATOR.as_str(),
            "BYPASS_UNH_EST"
        );
        assert_eq!(ParamId::PARAM_EST_ANGLE_LOCKOUT.as_str(), "EST_ANG_LOCKOUT");
        assert_eq!(ParamId::PARAM_ELEVATOR_REVERSE.as_str(), "REV_ELEVATOR");
        assert_eq!(ParamId::PARAM_IMU_ROLL.as_str(), "IMU_ROLL");
        assert_eq!(ParamId::PARAM_MAG_YAW.as_str(), "MAG_YAW");
        assert_eq!(ParamId::PARAM_ARM_THRESHOLD.as_str(), "RC_ARM_THRESHOLD");
        assert_eq!(
            ParamId::PARAM_BATTERY_VOLTAGE_ALPHA.as_str(),
            "BATT_VOLT_LPF"
        );
        assert_eq!(
            ParamId::PARAM_CHANNEL_OUTPUT_MASK.as_str(),
            "CHN_OUTPUT_MASK"
        );
        assert_eq!(
            ParamId::PARAM_RC_OUTPUT_KILL_CHANNEL.as_str(),
            "RC_KILL_CHN"
        );
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
            ParamValue::Int(1)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK),
            ParamValue::Int(0x0f)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_RC_OUTPUT_KILL_CHANNEL),
            ParamValue::Int(-1)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM),
            ParamValue::Int(0)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_BATTERY_VOLTAGE_MULTIPLIER),
            ParamValue::Float(1.0)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_TELEM_HEARTBEAT_HZ),
            ParamValue::Int(1)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_TELEM_STATUS_HZ),
            ParamValue::Int(10)
        );
        assert_eq!(
            p.get_by_id(ParamId::PARAM_TELEM_OUTPUT_RAW_HZ),
            ParamValue::Int(50)
        );
        for id in [
            ParamId::PARAM_TELEM_IMU_HZ,
            ParamId::PARAM_TELEM_ATTITUDE_HZ,
            ParamId::PARAM_TELEM_DIFF_PRESSURE_HZ,
            ParamId::PARAM_TELEM_BARO_HZ,
            ParamId::PARAM_TELEM_MAG_HZ,
            ParamId::PARAM_TELEM_RANGE_HZ,
            ParamId::PARAM_TELEM_BATTERY_HZ,
            ParamId::PARAM_TELEM_GNSS_HZ,
            ParamId::PARAM_TELEM_RC_HZ,
        ] {
            assert_eq!(p.get_by_id(id), ParamValue::Int(0));
        }
    }

    #[test]
    fn test_get_set_by_id() {
        let mut p = Params::new();
        p.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        assert_eq!(p.get_by_id(ParamId::PARAM_SYSTEM_ID), ParamValue::Int(42));
        p.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
        assert_eq!(p.get_by_id(ParamId::PARAM_FIXED_WING), ParamValue::Int(1));
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
}
