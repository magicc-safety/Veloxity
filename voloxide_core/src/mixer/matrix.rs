use crate::command::{
    OVERRIDE_ATT_SWITCH, OVERRIDE_OFFBOARD_T_INACTIVE, OVERRIDE_OFFBOARD_X_INACTIVE,
    OVERRIDE_OFFBOARD_Y_INACTIVE, OVERRIDE_OFFBOARD_Z_INACTIVE, OVERRIDE_T, OVERRIDE_THR_SWITCH,
    OVERRIDE_X, OVERRIDE_Y, OVERRIDE_Z,
};
use crate::controller::quad::ControllerOutput;
use crate::mixer::{Mixer, MixerCtx, MixerOutputType, MixerRun, MixerStatus};
use crate::params::{ParamId, ParamValue, Params};
use libm::{fabs, sqrt};
use nalgebra::SMatrix;

const NUM_MIXER_OUTPUTS: usize = 10;
const NUM_MIXERS: i32 = 12;
const ESC_CALIBRATION_MIXER: i32 = 0;
const QUAD_PLUS_MIXER: i32 = 1;
const QUAD_X_MIXER_CHOICE: i32 = 2;
const HEX_PLUS_MIXER: i32 = 3;
const HEX_X_MIXER: i32 = 4;
const OCTO_PLUS_MIXER: i32 = 5;
const OCTO_X_MIXER: i32 = 6;
const Y6_MIXER: i32 = 7;
const X8_MIXER: i32 = 8;
const FIXEDWING_MIXER: i32 = 9;
const INVERTED_VTAIL_MIXER: i32 = 10;
const CUSTOM_MIXER: i32 = 11;
const X_OVERRIDDEN: u16 = OVERRIDE_ATT_SWITCH | OVERRIDE_X | OVERRIDE_OFFBOARD_X_INACTIVE;
const Y_OVERRIDDEN: u16 = OVERRIDE_ATT_SWITCH | OVERRIDE_Y | OVERRIDE_OFFBOARD_Y_INACTIVE;
const Z_OVERRIDDEN: u16 = OVERRIDE_ATT_SWITCH | OVERRIDE_Z | OVERRIDE_OFFBOARD_Z_INACTIVE;
const T_OVERRIDDEN: u16 = OVERRIDE_THR_SWITCH | OVERRIDE_T | OVERRIDE_OFFBOARD_T_INACTIVE;
const ATTITUDE_OVERRIDDEN: u16 = X_OVERRIDDEN | Y_OVERRIDDEN | Z_OVERRIDDEN;
const QUAD_X_OUTPUT_TYPES: [MixerOutputType; NUM_MIXER_OUTPUTS] = [
    MixerOutputType::Motor,
    MixerOutputType::Motor,
    MixerOutputType::Motor,
    MixerOutputType::Motor,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
];
const ALL_MOTOR_OUTPUT_TYPES: [MixerOutputType; NUM_MIXER_OUTPUTS] =
    [MixerOutputType::Motor; NUM_MIXER_OUTPUTS];
const FIXEDWING_OUTPUT_TYPES: [MixerOutputType; NUM_MIXER_OUTPUTS] = [
    MixerOutputType::Servo,
    MixerOutputType::Servo,
    MixerOutputType::Servo,
    MixerOutputType::Aux,
    MixerOutputType::Motor,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
    MixerOutputType::Aux,
];
const ESC_CALIBRATION_PWM_RATES: [f64; NUM_MIXER_OUTPUTS] = [50.0; NUM_MIXER_OUTPUTS];
const QUAD_X_PWM_RATES: [f64; NUM_MIXER_OUTPUTS] = [
    490.0, 490.0, 490.0, 490.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0,
];
const HEX_PWM_RATES: [f64; NUM_MIXER_OUTPUTS] = [
    490.0, 490.0, 490.0, 490.0, 490.0, 490.0, 490.0, 490.0, 50.0, 50.0,
];
const OCTO_PWM_RATES: [f64; NUM_MIXER_OUTPUTS] = [
    490.0, 490.0, 490.0, 490.0, 490.0, 490.0, 490.0, 490.0, 50.0, 50.0,
];
const FIXEDWING_PWM_RATES: [f64; NUM_MIXER_OUTPUTS] = [50.0; NUM_MIXER_OUTPUTS];

#[rustfmt::skip]
const QUAD_X_MIXER: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [ 0.0000,  0.0000,  0.0000,  0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], // Fx
    [ 0.0000,  0.0000,  0.0000,  0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], // Fy
    [-0.2500, -0.2500, -0.2500, -0.2500, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], // Fz
    [-0.7071, -0.7071,  0.7071,  0.7071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], // Qx
    [ 0.7071, -0.7071, -0.7071,  0.7071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], // Qy
    [ 1.0000, -1.0000,  1.0000, -1.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], // Qz
    [ 0.0000,  0.0000,  0.0000,  0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [ 0.0000,  0.0000,  0.0000,  0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [ 0.0000,  0.0000,  0.0000,  0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [ 0.0000,  0.0000,  0.0000,  0.0000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const ESC_CALIBRATION_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const QUAD_PLUS_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.25, -0.25, -0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const FIXEDWING_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const INVERTED_VTAIL_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const HEX_PLUS_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.1667, -0.1667, -0.1667, -0.1667, -0.1667, -0.1667, 0.0, 0.0, 0.0, 0.0],
    [0.0, -0.8660, -0.8660, 0.0, 0.8660, 0.8660, 0.0, 0.0, 0.0, 0.0],
    [1.0, 0.5, -0.5, -1.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const HEX_X_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.1667, -0.1667, -0.1667, -0.1667, -0.1667, -0.1667, 0.0, 0.0, 0.0, 0.0],
    [-0.5, -1.0, -0.5, 0.5, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0],
    [0.8660, 0.0, -0.8660, -0.8660, 0.0, 0.8660, 0.0, 0.0, 0.0, 0.0],
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const OCTO_PLUS_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, 0.0, 0.0],
    [0.0, -0.7071, -1.0, -0.7071, 0.0, 0.7071, 1.0, 0.7071, 0.0, 0.0],
    [1.0, 0.7071, 0.0, -0.7071, -1.0, -0.7071, 0.0, 0.7071, 0.0, 0.0],
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const OCTO_X_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, 0.0, 0.0],
    [-0.3827, -0.9239, -0.9239, -0.3827, 0.3827, 0.9239, 0.9239, 0.3827, 0.0, 0.0],
    [0.9239, 0.3827, -0.3827, -0.9239, -0.9239, -0.3827, 0.3827, 0.9239, 0.0, 0.0],
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const Y6_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.1667, -0.1667, -0.1667, -0.1667, -0.1667, -0.1667, 0.0, 0.0, 0.0, 0.0],
    [-0.8660, -0.8660, 0.0, 0.0, 0.8660, 0.8660, 0.0, 0.0, 0.0, 0.0],
    [0.5, 0.5, -1.0, -1.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[rustfmt::skip]
const X8_MATRIX: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, -0.1250, 0.0, 0.0],
    [-0.7071, -0.7071, -0.7071, -0.7071, 0.7071, 0.7071, 0.7071, 0.7071, 0.0, 0.0],
    [0.7071, 0.7071, -0.7071, -0.7071, -0.7071, -0.7071, 0.7071, 0.7071, 0.0, 0.0],
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

#[derive(Debug, Clone, Copy)]
pub struct MixerParams {
    // Safety / limits
    pub idle_throttle: f64,
    pub spin_when_armed: bool,
    pub num_motors: usize,
}

pub struct MatrixMixer {
    params: MixerParams,
    output_types: [MixerOutputType; NUM_MIXER_OUTPUTS],
    default_pwm_rates: [f64; NUM_MIXER_OUTPUTS],
    primary_mixer: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
    secondary_mixer: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
}

#[derive(Clone, Copy)]
struct CannedMixerConfig {
    output_types: [MixerOutputType; NUM_MIXER_OUTPUTS],
    default_pwm_rates: [f64; NUM_MIXER_OUTPUTS],
    matrix: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
}

impl MatrixMixer {
    pub fn new(params: &Params) -> Self {
        let mixer_params = MixerParams {
            num_motors: 4,
            idle_throttle: if let ParamValue::Float(v) =
                params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE)
            {
                v as f64
            } else {
                0.1
            },
            spin_when_armed: if let ParamValue::Int(v) =
                params.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED)
            {
                v != 0
            } else {
                true
            },
        };

        let mut mixer = Self {
            params: mixer_params,
            output_types: QUAD_X_OUTPUT_TYPES,
            default_pwm_rates: [50.0; NUM_MIXER_OUTPUTS],
            primary_mixer: QUAD_X_MIXER,
            secondary_mixer: QUAD_X_MIXER,
        };
        mixer.refresh_mixer_config(params);
        mixer
    }
}

impl Mixer for MatrixMixer {
    type MixerInput = ControllerOutput;
    type ActuatorCommands = [f64; NUM_MIXER_OUTPUTS];

    fn mix(
        &mut self,
        controls: &Self::MixerInput,
        ctx: MixerCtx<'_>,
    ) -> MixerRun<Self::ActuatorCommands> {
        let status = self.refresh_mixer_config(ctx.params);

        if matches!(status, MixerStatus::InvalidMixer) {
            return MixerRun {
                commands: [0.0; NUM_MIXER_OUTPUTS],
                status,
            };
        }

        let mut commands = *controls;
        self.params.idle_throttle = param_float(ctx.params, ParamId::PARAM_MOTOR_IDLE_THROTTLE);
        self.params.spin_when_armed =
            param_int(ctx.params, ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED) != 0;
        self.params.num_motors = param_int(ctx.params, ParamId::PARAM_NUM_MOTORS).max(0) as usize;

        let fixed_wing = param_int(ctx.params, ParamId::PARAM_FIXED_WING) != 0
            || matches!(
                param_int(ctx.params, ParamId::PARAM_PRIMARY_MIXER),
                FIXEDWING_MIXER | INVERTED_VTAIL_MIXER
            );

        if fixed_wing {
            apply_fixedwing_reversals(&mut commands, ctx.params);
        } else if fabs(throttle_command(&commands, ctx.params)) < self.params.idle_throttle {
            commands.u[5] = 0.0;
        }

        let mixer_to_use = self.select_primary_or_secondary(ctx.rc_override);
        let use_motor_parameters =
            !fixed_wing && param_int(ctx.params, ParamId::PARAM_USE_MOTOR_PARAMETERS) != 0;
        let mut outputs = [0.0; NUM_MIXER_OUTPUTS];
        let mut max_output = 1.0;

        for output in 0..NUM_MIXER_OUTPUTS {
            if self.output_types[output] == MixerOutputType::Aux {
                continue;
            }

            let value =
                if use_motor_parameters && self.output_types[output] == MixerOutputType::Motor {
                    self.mix_motor_parameter_output(output, &commands, mixer_to_use, &ctx)
                } else {
                    matrix_output(output, &commands, mixer_to_use)
                };
            outputs[output] = value;

            if self.output_types[output] == MixerOutputType::Motor && fabs(value) > max_output {
                max_output = fabs(value);
            }
        }

        if max_output > 2.0 {
            crate::log_warn!("Output from mixer is {}! Check mixer", max_output);
        }

        let scale_factor = if max_output > 1.0 {
            1.0 / max_output
        } else {
            1.0
        };

        for (output, value) in outputs.iter_mut().enumerate() {
            if self.output_types[output] != MixerOutputType::Motor {
                continue;
            }

            *value *= scale_factor;
            if *value > 1.0 {
                *value = 1.0;
            } else if *value < self.params.idle_throttle && self.params.spin_when_armed {
                *value = self.params.idle_throttle;
            } else if *value < 0.0 {
                *value = 0.0;
            }
        }

        MixerRun {
            commands: outputs,
            status,
        }
    }

    fn output_types(&self) -> &[MixerOutputType] {
        &self.output_types
    }

    fn default_pwm_rates(&self) -> &[f64] {
        &self.default_pwm_rates
    }
}

impl MatrixMixer {
    fn refresh_mixer_config(&mut self, params: &Params) -> MixerStatus {
        let primary_choice = param_int(params, ParamId::PARAM_PRIMARY_MIXER);
        if primary_choice >= NUM_MIXERS {
            crate::log_error!("Invalid mixer choice for primary mixer");
            return MixerStatus::InvalidMixer;
        }

        if let Some(config) = canned_mixer(primary_choice) {
            self.output_types = config.output_types;
            self.default_pwm_rates = config.default_pwm_rates;
            self.primary_mixer = config.matrix;
        } else {
            self.output_types = output_types_from_params(params).unwrap_or(QUAD_X_OUTPUT_TYPES);
            self.default_pwm_rates =
                pwm_rates_from_params(params).unwrap_or([50.0; NUM_MIXER_OUTPUTS]);
            self.primary_mixer = mixer_from_params(
                params,
                ParamId::PARAM_PRIMARY_MIXER,
                ParamId::PARAM_PRIMARY_MIXER_0_0,
            )
            .unwrap_or(QUAD_X_MIXER);
        }

        let secondary_choice = param_int(params, ParamId::PARAM_SECONDARY_MIXER);
        self.secondary_mixer = if let Some(config) = canned_secondary_mixer(secondary_choice) {
            config.matrix
        } else {
            mixer_from_params(
                params,
                ParamId::PARAM_SECONDARY_MIXER,
                ParamId::PARAM_SECONDARY_MIXER_0_0,
            )
            .unwrap_or(self.primary_mixer)
        };

        MixerStatus::Healthy
    }

    fn select_primary_or_secondary(
        &self,
        rc_override: u16,
    ) -> [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] {
        let mut mixer = self.secondary_mixer;

        if rc_override & ATTITUDE_OVERRIDDEN != 0 {
            mixer[3] = self.primary_mixer[3];
            mixer[4] = self.primary_mixer[4];
            mixer[5] = self.primary_mixer[5];
        }

        if rc_override & T_OVERRIDDEN != 0 {
            mixer[0] = self.primary_mixer[0];
            mixer[1] = self.primary_mixer[1];
            mixer[2] = self.primary_mixer[2];
        }

        mixer
    }

    fn mix_motor_parameter_output(
        &self,
        output: usize,
        commands: &ControllerOutput,
        mixer: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
        ctx: &MixerCtx<'_>,
    ) -> f64 {
        let omega_squared = matrix_output(output, commands, mixer).max(0.0);
        let k_q = param_float(ctx.params, ParamId::PARAM_MOTOR_KV);
        if k_q < 0.0000001 {
            return 0.0;
        }

        let Some(battery_voltage) = ctx.battery_voltage else {
            return 0.0;
        };
        if battery_voltage < 0.0001 {
            return 0.0;
        }

        let resistance = param_float(ctx.params, ParamId::PARAM_MOTOR_RESISTANCE);
        let diameter = param_float(ctx.params, ParamId::PARAM_PROP_DIAMETER);
        let cq = param_float(ctx.params, ParamId::PARAM_PROP_CQ);
        let kv = param_float(ctx.params, ParamId::PARAM_MOTOR_KV);
        let no_load_current = param_float(ctx.params, ParamId::PARAM_NO_LOAD_CURRENT);

        let diameter_5 = diameter * diameter * diameter * diameter * diameter;
        let pi_2 = core::f64::consts::PI * core::f64::consts::PI;
        let voltage = ctx.air_density * diameter_5 / (4.0 * pi_2) * omega_squared * cq * resistance
            / k_q
            + resistance * no_load_current
            + kv * sqrt(omega_squared);

        voltage / battery_voltage
    }
}

fn matrix_output(
    output: usize,
    commands: &ControllerOutput,
    mixer: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
) -> f64 {
    let mut value = 0.0;
    for input in 0..NUM_MIXER_OUTPUTS {
        value += commands.u[input] * mixer[input][output];
    }
    value
}

fn throttle_command(commands: &ControllerOutput, params: &Params) -> f64 {
    match param_int(params, ParamId::PARAM_RC_F_AXIS) {
        0 => commands.u[0],
        1 => commands.u[1],
        _ => commands.u[2],
    }
}

fn param_float(params: &Params, id: ParamId) -> f64 {
    match params.get_by_id(id) {
        ParamValue::Float(value) => value as f64,
        _ => 0.0,
    }
}

fn param_int(params: &Params, id: ParamId) -> i32 {
    match params.get_by_id(id) {
        ParamValue::Int(value) => value,
        _ => 0,
    }
}

fn mixer_from_params(
    params: &Params,
    mixer_choice_id: ParamId,
    first_matrix_id: ParamId,
) -> Option<[[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS]> {
    if param_int(params, mixer_choice_id) != CUSTOM_MIXER {
        return None;
    }

    let mut mixer = [[0.0; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS];
    let first_index = first_matrix_id as usize;
    for col in 0..NUM_MIXER_OUTPUTS {
        for (row, mixer_row) in mixer.iter_mut().enumerate() {
            let param_index = first_index + col * NUM_MIXER_OUTPUTS + row;
            let Some(param_id) = ParamId::from_index(param_index) else {
                return None;
            };
            mixer_row[col] = param_float(params, param_id);
        }
    }

    Some(mixer)
}

fn output_types_from_params(params: &Params) -> Option<[MixerOutputType; NUM_MIXER_OUTPUTS]> {
    if param_int(params, ParamId::PARAM_PRIMARY_MIXER) != CUSTOM_MIXER {
        return None;
    }

    let mut output_types = [MixerOutputType::Aux; NUM_MIXER_OUTPUTS];
    let first_index = ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0 as usize;
    for (output, output_type) in output_types.iter_mut().enumerate() {
        let param_id = ParamId::from_index(first_index + output)?;
        *output_type = mixer_output_type_from_rosflight(param_int(params, param_id));
    }

    Some(output_types)
}

fn pwm_rates_from_params(params: &Params) -> Option<[f64; NUM_MIXER_OUTPUTS]> {
    if param_int(params, ParamId::PARAM_PRIMARY_MIXER) != CUSTOM_MIXER {
        return None;
    }

    let mut rates = [0.0; NUM_MIXER_OUTPUTS];
    let first_index = ParamId::PARAM_PRIMARY_MIXER_PWM_RATE_0 as usize;
    for (output, rate) in rates.iter_mut().enumerate() {
        let param_id = ParamId::from_index(first_index + output)?;
        *rate = param_float(params, param_id);
    }

    Some(rates)
}

fn mixer_output_type_from_rosflight(value: i32) -> MixerOutputType {
    match value {
        1 => MixerOutputType::Servo,
        2 => MixerOutputType::Motor,
        3 => MixerOutputType::Gpio,
        _ => MixerOutputType::Aux,
    }
}

fn mixer_output_type_to_rosflight(value: MixerOutputType) -> i32 {
    match value {
        MixerOutputType::Aux => 0,
        MixerOutputType::Servo => 1,
        MixerOutputType::Motor => 2,
        MixerOutputType::Gpio => 3,
    }
}

pub fn sync_reflected_mixer_params(params: &mut Params, changed: ParamId) {
    match changed {
        ParamId::PARAM_PRIMARY_MIXER => {
            let choice = param_int(params, ParamId::PARAM_PRIMARY_MIXER);
            let Some(config) = canned_mixer(choice) else {
                return;
            };
            save_primary_mixer_params(params, config);
            let secondary_choice = param_int(params, ParamId::PARAM_SECONDARY_MIXER);
            if !(0..NUM_MIXERS).contains(&secondary_choice) {
                save_secondary_mixer_params(params, config.matrix);
            }
        }
        ParamId::PARAM_SECONDARY_MIXER => {
            let choice = param_int(params, ParamId::PARAM_SECONDARY_MIXER);
            let Some(config) = canned_secondary_mixer(choice) else {
                return;
            };
            save_secondary_mixer_params(params, config.matrix);
        }
        _ => {}
    }
}

fn save_primary_mixer_params(params: &mut Params, config: CannedMixerConfig) {
    for output in 0..NUM_MIXER_OUTPUTS {
        if let Some(output_id) =
            ParamId::from_index(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0 as usize + output)
        {
            params.set_by_id(
                output_id,
                ParamValue::Int(mixer_output_type_to_rosflight(config.output_types[output])),
            );
        }

        if let Some(rate_id) =
            ParamId::from_index(ParamId::PARAM_PRIMARY_MIXER_PWM_RATE_0 as usize + output)
        {
            params.set_by_id(
                rate_id,
                ParamValue::Float(config.default_pwm_rates[output] as f32),
            );
        }
    }

    save_mixer_matrix_params(params, ParamId::PARAM_PRIMARY_MIXER_0_0, config.matrix);
}

fn save_secondary_mixer_params(
    params: &mut Params,
    matrix: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
) {
    save_mixer_matrix_params(params, ParamId::PARAM_SECONDARY_MIXER_0_0, matrix);
}

fn save_mixer_matrix_params(
    params: &mut Params,
    first_matrix_id: ParamId,
    matrix: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
) {
    let first_index = first_matrix_id as usize;
    for col in 0..NUM_MIXER_OUTPUTS {
        for (row, mixer_row) in matrix.iter().enumerate() {
            let Some(param_id) = ParamId::from_index(first_index + col * NUM_MIXER_OUTPUTS + row)
            else {
                return;
            };
            params.set_by_id(param_id, ParamValue::Float(mixer_row[col] as f32));
        }
    }
}

fn inverted_multirotor_mixer(
    mixer: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
) -> [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] {
    let mut mixer_matrix = SMatrix::<f64, NUM_MIXER_OUTPUTS, NUM_MIXER_OUTPUTS>::zeros();
    for row in 0..NUM_MIXER_OUTPUTS {
        for col in 0..NUM_MIXER_OUTPUTS {
            mixer_matrix[(row, col)] = mixer[row][col];
        }
    }

    let svd = mixer_matrix.svd(true, true);
    let Some(u) = svd.u else {
        return mixer;
    };
    let Some(v_t) = svd.v_t else {
        return mixer;
    };

    let mut sigma_inverse = SMatrix::<f64, NUM_MIXER_OUTPUTS, NUM_MIXER_OUTPUTS>::zeros();
    for i in 0..NUM_MIXER_OUTPUTS {
        let singular_value = svd.singular_values[i];
        if singular_value != 0.0 {
            sigma_inverse[(i, i)] = 1.0 / singular_value;
        }
    }

    let mixer_matrix_pinv = v_t.transpose() * sigma_inverse * u.transpose();
    let mut inverted = [[0.0; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS];
    for i in 0..NUM_MIXER_OUTPUTS {
        for j in 0..NUM_MIXER_OUTPUTS {
            inverted[j][i] = mixer_matrix_pinv[(i, j)];
        }
    }
    inverted
}

fn rank_one_pseudoinverse_mixer(
    mixer: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
) -> [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] {
    let mut norm_squared = 0.0;
    for row in mixer.iter() {
        for value in row.iter() {
            norm_squared += value * value;
        }
    }

    if norm_squared < 1.0e-12 {
        return mixer;
    }

    let mut inverted = [[0.0; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS];
    for row in 0..NUM_MIXER_OUTPUTS {
        for col in 0..NUM_MIXER_OUTPUTS {
            inverted[row][col] = mixer[row][col] / norm_squared;
        }
    }

    inverted
}

fn canned_mixer(choice: i32) -> Option<CannedMixerConfig> {
    let config = match choice {
        ESC_CALIBRATION_MIXER => CannedMixerConfig {
            output_types: ALL_MOTOR_OUTPUT_TYPES,
            default_pwm_rates: ESC_CALIBRATION_PWM_RATES,
            matrix: rank_one_pseudoinverse_mixer(ESC_CALIBRATION_MATRIX),
        },
        QUAD_PLUS_MIXER => CannedMixerConfig {
            output_types: QUAD_X_OUTPUT_TYPES,
            default_pwm_rates: QUAD_X_PWM_RATES,
            matrix: inverted_multirotor_mixer(QUAD_PLUS_MATRIX),
        },
        QUAD_X_MIXER_CHOICE => CannedMixerConfig {
            output_types: QUAD_X_OUTPUT_TYPES,
            default_pwm_rates: QUAD_X_PWM_RATES,
            matrix: inverted_multirotor_mixer(QUAD_X_MIXER),
        },
        HEX_PLUS_MIXER => CannedMixerConfig {
            output_types: [
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
            ],
            default_pwm_rates: HEX_PWM_RATES,
            matrix: inverted_multirotor_mixer(HEX_PLUS_MATRIX),
        },
        HEX_X_MIXER => CannedMixerConfig {
            output_types: [
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
            ],
            default_pwm_rates: HEX_PWM_RATES,
            matrix: inverted_multirotor_mixer(HEX_X_MATRIX),
        },
        Y6_MIXER => CannedMixerConfig {
            output_types: [
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
            ],
            default_pwm_rates: HEX_PWM_RATES,
            matrix: inverted_multirotor_mixer(Y6_MATRIX),
        },
        OCTO_PLUS_MIXER => CannedMixerConfig {
            output_types: [
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
            ],
            default_pwm_rates: OCTO_PWM_RATES,
            matrix: inverted_multirotor_mixer(OCTO_PLUS_MATRIX),
        },
        OCTO_X_MIXER => CannedMixerConfig {
            output_types: [
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
            ],
            default_pwm_rates: OCTO_PWM_RATES,
            matrix: inverted_multirotor_mixer(OCTO_X_MATRIX),
        },
        X8_MIXER => CannedMixerConfig {
            output_types: [
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Motor,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
            ],
            default_pwm_rates: OCTO_PWM_RATES,
            matrix: inverted_multirotor_mixer(X8_MATRIX),
        },
        FIXEDWING_MIXER => CannedMixerConfig {
            output_types: FIXEDWING_OUTPUT_TYPES,
            default_pwm_rates: FIXEDWING_PWM_RATES,
            matrix: FIXEDWING_MATRIX,
        },
        INVERTED_VTAIL_MIXER => CannedMixerConfig {
            output_types: FIXEDWING_OUTPUT_TYPES,
            default_pwm_rates: FIXEDWING_PWM_RATES,
            matrix: INVERTED_VTAIL_MATRIX,
        },
        _ => return None,
    };

    Some(config)
}

fn canned_secondary_mixer(choice: i32) -> Option<CannedMixerConfig> {
    let mut config = canned_mixer(choice)?;
    config.matrix = match choice {
        CUSTOM_MIXER => config.matrix,
        _ => inverted_multirotor_mixer(raw_canned_mixer_matrix(choice)?),
    };
    Some(config)
}

fn raw_canned_mixer_matrix(choice: i32) -> Option<[[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS]> {
    match choice {
        ESC_CALIBRATION_MIXER => Some(ESC_CALIBRATION_MATRIX),
        QUAD_PLUS_MIXER => Some(QUAD_PLUS_MATRIX),
        QUAD_X_MIXER_CHOICE => Some(QUAD_X_MIXER),
        HEX_PLUS_MIXER => Some(HEX_PLUS_MATRIX),
        HEX_X_MIXER => Some(HEX_X_MATRIX),
        OCTO_PLUS_MIXER => Some(OCTO_PLUS_MATRIX),
        OCTO_X_MIXER => Some(OCTO_X_MATRIX),
        Y6_MIXER => Some(Y6_MATRIX),
        X8_MIXER => Some(X8_MATRIX),
        FIXEDWING_MIXER => Some(FIXEDWING_MATRIX),
        INVERTED_VTAIL_MIXER => Some(INVERTED_VTAIL_MATRIX),
        _ => None,
    }
}

fn apply_fixedwing_reversals(commands: &mut ControllerOutput, params: &Params) {
    if param_int(params, ParamId::PARAM_AILERON_REVERSE) != 0 {
        commands.u[3] *= -1.0;
    }
    if param_int(params, ParamId::PARAM_ELEVATOR_REVERSE) != 0 {
        commands.u[4] *= -1.0;
    }
    if param_int(params, ParamId::PARAM_RUDDER_REVERSE) != 0 {
        commands.u[5] *= -1.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::{OVERRIDE_ATT_SWITCH, OVERRIDE_NO_OVERRIDE, OVERRIDE_THR_SWITCH};
    use crate::state_machine::StateManager;

    fn mixer_ctx<'a>(
        state: &'a StateManager,
        params: &'a Params,
        rc_override: u16,
    ) -> MixerCtx<'a> {
        MixerCtx {
            state,
            params,
            rc_override,
            air_density: 1.225,
            battery_voltage: None,
        }
    }

    #[test]
    fn canned_fixedwing_mixer_maps_controls_and_pwm_rates_like_rosflight_c() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
        params.set_by_id(
            ParamId::PARAM_PRIMARY_MIXER,
            ParamValue::Int(FIXEDWING_MIXER),
        );
        let state = StateManager::new();
        let mut mixer = MatrixMixer::new(&params);
        let mut controls = ControllerOutput::default();
        controls.u[0] = 0.7;
        controls.u[3] = 0.1;
        controls.u[4] = -0.2;
        controls.u[5] = 0.3;

        let run = mixer.mix(&controls, mixer_ctx(&state, &params, OVERRIDE_NO_OVERRIDE));

        assert_eq!(run.status, MixerStatus::Healthy);
        assert_eq!(
            mixer.output_types(),
            [
                MixerOutputType::Servo,
                MixerOutputType::Servo,
                MixerOutputType::Servo,
                MixerOutputType::Aux,
                MixerOutputType::Motor,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
                MixerOutputType::Aux,
            ]
        );
        assert_eq!(mixer.default_pwm_rates(), [50.0; NUM_MIXER_OUTPUTS]);
        assert!((run.commands[0] - 0.1).abs() < 1e-9);
        assert!((run.commands[1] + 0.2).abs() < 1e-9);
        assert!((run.commands[2] - 0.3).abs() < 1e-9);
        assert_eq!(run.commands[3], 0.0);
        assert!((run.commands[4] - 0.7).abs() < 1e-9);
    }

    #[test]
    fn canned_fixedwing_reversal_params_apply_before_mixing() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
        params.set_by_id(
            ParamId::PARAM_PRIMARY_MIXER,
            ParamValue::Int(FIXEDWING_MIXER),
        );
        params.set_by_id(ParamId::PARAM_AILERON_REVERSE, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_ELEVATOR_REVERSE, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_RUDDER_REVERSE, ParamValue::Int(1));
        let state = StateManager::new();
        let mut mixer = MatrixMixer::new(&params);
        let mut controls = ControllerOutput::default();
        controls.u[3] = 0.1;
        controls.u[4] = -0.2;
        controls.u[5] = 0.3;

        let run = mixer.mix(&controls, mixer_ctx(&state, &params, OVERRIDE_NO_OVERRIDE));

        assert!((run.commands[0] + 0.1).abs() < 1e-9);
        assert!((run.commands[1] - 0.2).abs() < 1e-9);
        assert!((run.commands[2] + 0.3).abs() < 1e-9);
    }

    #[test]
    fn secondary_mixer_row_selection_matches_rosflight_c_override_masks() {
        let mut params = Params::new();
        params.set_by_id(
            ParamId::PARAM_PRIMARY_MIXER,
            ParamValue::Int(FIXEDWING_MIXER),
        );
        params.set_by_id(
            ParamId::PARAM_SECONDARY_MIXER,
            ParamValue::Int(INVERTED_VTAIL_MIXER),
        );
        let mut mixer = MatrixMixer::new(&params);
        assert_eq!(mixer.refresh_mixer_config(&params), MixerStatus::Healthy);

        let secondary_only = mixer.select_primary_or_secondary(OVERRIDE_NO_OVERRIDE);
        assert_eq!(secondary_only[0], mixer.secondary_mixer[0]);
        assert_eq!(secondary_only[3], mixer.secondary_mixer[3]);

        let attitude_override = mixer.select_primary_or_secondary(OVERRIDE_ATT_SWITCH);
        assert_eq!(attitude_override[0], mixer.secondary_mixer[0]);
        assert_eq!(attitude_override[1], mixer.secondary_mixer[1]);
        assert_eq!(attitude_override[2], mixer.secondary_mixer[2]);
        assert_eq!(attitude_override[3], mixer.primary_mixer[3]);
        assert_eq!(attitude_override[4], mixer.primary_mixer[4]);
        assert_eq!(attitude_override[5], mixer.primary_mixer[5]);

        let throttle_override = mixer.select_primary_or_secondary(OVERRIDE_THR_SWITCH);
        assert_eq!(throttle_override[0], mixer.primary_mixer[0]);
        assert_eq!(throttle_override[1], mixer.primary_mixer[1]);
        assert_eq!(throttle_override[2], mixer.primary_mixer[2]);
        assert_eq!(throttle_override[3], mixer.secondary_mixer[3]);
        assert_eq!(throttle_override[4], mixer.secondary_mixer[4]);
        assert_eq!(throttle_override[5], mixer.secondary_mixer[5]);
    }

    #[test]
    fn reflected_secondary_mixer_defaults_to_primary_when_choice_is_unset() {
        let mut params = Params::new();
        params.set_by_id(
            ParamId::PARAM_PRIMARY_MIXER,
            ParamValue::Int(INVERTED_VTAIL_MIXER),
        );
        params.set_by_id(ParamId::PARAM_SECONDARY_MIXER, ParamValue::Int(-1));

        sync_reflected_mixer_params(&mut params, ParamId::PARAM_PRIMARY_MIXER);

        assert_eq!(
            params.get_by_id(ParamId::PARAM_PRIMARY_MIXER_3_0),
            params.get_by_id(ParamId::PARAM_SECONDARY_MIXER_3_0)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_PRIMARY_MIXER_4_1),
            params.get_by_id(ParamId::PARAM_SECONDARY_MIXER_4_1)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_PRIMARY_MIXER_5_1),
            params.get_by_id(ParamId::PARAM_SECONDARY_MIXER_5_1)
        );
    }
}
