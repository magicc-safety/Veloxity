use crate::command::{
    OVERRIDE_ATT_SWITCH, OVERRIDE_OFFBOARD_T_INACTIVE, OVERRIDE_OFFBOARD_X_INACTIVE,
    OVERRIDE_OFFBOARD_Y_INACTIVE, OVERRIDE_OFFBOARD_Z_INACTIVE, OVERRIDE_T, OVERRIDE_THR_SWITCH,
    OVERRIDE_X, OVERRIDE_Y, OVERRIDE_Z,
};
use crate::controller::quad::ControllerOutput;
use crate::math::prelude::*;
use crate::mixer::{Mixer, MixerCtx, MixerOutputType, MixerRun, MixerStatus};
use crate::params::{ParamId, ParamValue, Params};
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
pub struct MixerParams<R: FlightFloat> {
    // Safety / limits
    pub idle_throttle: R,
    pub num_motors: usize,
    pub fixed_wing: bool,
    pub use_motor_parameters: bool,
    pub throttle_axis: usize,
}

pub struct MatrixMixer<R: FlightFloat> {
    params: MixerParams<R>,
    status: MixerStatus,
    output_types: [MixerOutputType; NUM_MIXER_OUTPUTS],
    default_pwm_rates: [R; NUM_MIXER_OUTPUTS],
    primary_mixer: [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
    secondary_mixer: [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
}

#[derive(Clone, Copy)]
struct CannedMixerConfig<R: FlightFloat> {
    output_types: [MixerOutputType; NUM_MIXER_OUTPUTS],
    default_pwm_rates: [R; NUM_MIXER_OUTPUTS],
    matrix: [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
}

impl<R: FlightFloat> MatrixMixer<R> {
    pub fn new(params: &Params) -> Self {
        let mixer_params = MixerParams {
            num_motors: 4,
            idle_throttle: if let ParamValue::Float(v) =
                params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE)
            {
                <R as FlightFloat>::from_f32(v)
            } else {
                <R as FlightFloat>::from_f32(0.1)
            },
            fixed_wing: false,
            use_motor_parameters: false,
            throttle_axis: 2,
        };

        let mut mixer = Self {
            params: mixer_params,
            status: MixerStatus::Healthy,
            output_types: QUAD_X_OUTPUT_TYPES,
            default_pwm_rates: [<R as FlightFloat>::from_f32(50.0); NUM_MIXER_OUTPUTS],
            primary_mixer: matrix_from_default(QUAD_X_MIXER),
            secondary_mixer: matrix_from_default(QUAD_X_MIXER),
        };
        mixer.status = mixer.refresh_mixer_config(params);
        mixer.refresh_runtime_params(params);
        mixer
    }
}

impl<R: FlightFloat> Mixer<R> for MatrixMixer<R> {
    type MixerInput = ControllerOutput<R>;
    type ActuatorCommands = [R; NUM_MIXER_OUTPUTS];

    fn mix(
        &mut self,
        controls: &Self::MixerInput,
        ctx: MixerCtx<'_, R>,
    ) -> MixerRun<Self::ActuatorCommands> {
        let status = self.status;

        if matches!(status, MixerStatus::InvalidMixer) {
            return MixerRun {
                commands: [<R as FlightFloat>::from_f32(0.0); NUM_MIXER_OUTPUTS],
                status,
            };
        }

        let mut commands = *controls;
        if self.params.fixed_wing {
            apply_fixedwing_reversals(&mut commands, ctx.params);
        } else if throttle_command(&commands, self.params.throttle_axis).abs()
            < self.params.idle_throttle
        {
            commands.u[5] = <R as FlightFloat>::from_f32(0.0);
        }

        let mut outputs = [<R as FlightFloat>::from_f32(0.0); NUM_MIXER_OUTPUTS];
        let mut max_output = <R as FlightFloat>::from_f32(1.0);

        if self.params.use_motor_parameters {
            for output in 0..NUM_MIXER_OUTPUTS {
                if self.output_types[output] == MixerOutputType::Aux {
                    continue;
                }

                let value = if self.output_types[output] == MixerOutputType::Motor {
                    self.mix_motor_parameter_output(output, &commands, ctx.rc_override, &ctx)
                } else {
                    self.matrix_output_selected(output, &commands, ctx.rc_override)
                };
                outputs[output] = value;

                if self.output_types[output] == MixerOutputType::Motor && value.abs() > max_output {
                    max_output = value.abs();
                }
            }
        } else {
            self.matrix_outputs_selected(&commands, ctx.rc_override, &mut outputs);
            for (output, value) in outputs.iter().enumerate() {
                if self.output_types[output] == MixerOutputType::Motor && value.abs() > max_output {
                    max_output = value.abs();
                }
            }
        }

        if max_output > <R as FlightFloat>::from_f32(2.0) {
            crate::log_warn!("Output from mixer is {}! Check mixer", max_output);
        }

        let scale_factor = if max_output > <R as FlightFloat>::from_f32(1.0) {
            <R as FlightFloat>::from_f32(1.0) / max_output
        } else {
            <R as FlightFloat>::from_f32(1.0)
        };

        for (output, value) in outputs.iter_mut().enumerate() {
            if self.output_types[output] != MixerOutputType::Motor {
                continue;
            }

            *value *= scale_factor;
        }

        MixerRun {
            commands: outputs,
            status,
        }
    }

    fn output_types(&self) -> &[MixerOutputType] {
        &self.output_types
    }

    fn default_pwm_rates(&self) -> &[R] {
        &self.default_pwm_rates
    }

    fn on_param_changed(&mut self, params: &Params, id: ParamId) -> Option<MixerStatus> {
        if is_mixer_runtime_param(id) {
            self.refresh_runtime_params(params);
        }
        if is_mixer_config_param(id) {
            self.status = self.refresh_mixer_config(params);
            self.refresh_runtime_params(params);
            Some(self.status)
        } else {
            None
        }
    }
}

impl<R: FlightFloat> MatrixMixer<R> {
    fn refresh_runtime_params(&mut self, params: &Params) {
        self.params.idle_throttle = param_float(params, ParamId::PARAM_MOTOR_IDLE_THROTTLE);
        self.params.num_motors = param_int(params, ParamId::PARAM_NUM_MOTORS).max(0) as usize;
        let primary_choice = param_int(params, ParamId::PARAM_PRIMARY_MIXER);
        self.params.fixed_wing = param_int(params, ParamId::PARAM_FIXED_WING) != 0
            || matches!(primary_choice, FIXEDWING_MIXER | INVERTED_VTAIL_MIXER);
        self.params.use_motor_parameters =
            !self.params.fixed_wing && param_int(params, ParamId::PARAM_USE_MOTOR_PARAMETERS) != 0;
        self.params.throttle_axis =
            param_int(params, ParamId::PARAM_RC_F_AXIS).clamp(0, 2) as usize;
    }

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
            self.default_pwm_rates = pwm_rates_from_params(params)
                .unwrap_or([<R as FlightFloat>::from_f32(50.0); NUM_MIXER_OUTPUTS]);
            self.primary_mixer = mixer_from_params(
                params,
                ParamId::PARAM_PRIMARY_MIXER,
                ParamId::PARAM_PRIMARY_MIXER_0_0,
            )
            .unwrap_or(matrix_from_default(QUAD_X_MIXER));
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

    #[cfg(test)]
    fn select_primary_or_secondary(
        &self,
        rc_override: u16,
    ) -> [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] {
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
        commands: &ControllerOutput<R>,
        rc_override: u16,
        ctx: &MixerCtx<'_, R>,
    ) -> R {
        let omega_squared = self
            .matrix_output_selected(output, commands, rc_override)
            .max(<R as FlightFloat>::from_f32(0.0));
        let k_q = param_float(ctx.params, ParamId::PARAM_MOTOR_KV);
        if k_q < <R as FlightFloat>::from_f32(0.0000001) {
            return <R as FlightFloat>::from_f32(0.0);
        }

        let Some(battery_voltage) = ctx.battery_voltage else {
            return <R as FlightFloat>::from_f32(0.0);
        };
        if battery_voltage < <R as FlightFloat>::from_f32(0.0001) {
            return <R as FlightFloat>::from_f32(0.0);
        }

        let resistance = param_float::<R>(ctx.params, ParamId::PARAM_MOTOR_RESISTANCE);
        let diameter = param_float::<R>(ctx.params, ParamId::PARAM_PROP_DIAMETER);
        let cq = param_float::<R>(ctx.params, ParamId::PARAM_PROP_CQ);
        let kv = param_float::<R>(ctx.params, ParamId::PARAM_MOTOR_KV);
        let no_load_current = param_float::<R>(ctx.params, ParamId::PARAM_NO_LOAD_CURRENT);

        let diameter_5 = diameter * diameter * diameter * diameter * diameter;
        let pi = pi::<R>();
        let pi_2 = pi * pi;
        let voltage = ctx.air_density * diameter_5 / (<R as FlightFloat>::from_f32(4.0) * pi_2)
            * omega_squared
            * cq
            * resistance
            / k_q
            + resistance * no_load_current
            + kv * omega_squared.sqrt();

        voltage / battery_voltage
    }

    fn matrix_output_selected(
        &self,
        output: usize,
        commands: &ControllerOutput<R>,
        rc_override: u16,
    ) -> R {
        let mut value = <R as FlightFloat>::from_f32(0.0);
        for input in 0..NUM_MIXER_OUTPUTS {
            let command = commands.u[input];
            if command == <R as FlightFloat>::from_f32(0.0) {
                continue;
            }
            let row = if self.use_primary_row_for_override(input, rc_override) {
                &self.primary_mixer[input]
            } else {
                &self.secondary_mixer[input]
            };
            value += command * row[output];
        }
        value
    }

    fn matrix_outputs_selected(
        &self,
        commands: &ControllerOutput<R>,
        rc_override: u16,
        outputs: &mut [R; NUM_MIXER_OUTPUTS],
    ) {
        for input in 0..NUM_MIXER_OUTPUTS {
            let command = commands.u[input];
            if command == <R as FlightFloat>::from_f32(0.0) {
                continue;
            }
            let row = if self.use_primary_row_for_override(input, rc_override) {
                &self.primary_mixer[input]
            } else {
                &self.secondary_mixer[input]
            };
            for output in 0..NUM_MIXER_OUTPUTS {
                if self.output_types[output] != MixerOutputType::Aux {
                    outputs[output] += command * row[output];
                }
            }
        }
    }

    fn use_primary_row_for_override(&self, input: usize, rc_override: u16) -> bool {
        ((3..=5).contains(&input) && rc_override & ATTITUDE_OVERRIDDEN != 0)
            || (input <= 2 && rc_override & T_OVERRIDDEN != 0)
    }
}

fn throttle_command<R: FlightFloat>(commands: &ControllerOutput<R>, throttle_axis: usize) -> R {
    commands.u[throttle_axis.min(2)]
}

fn param_float<R: FlightFloat>(params: &Params, id: ParamId) -> R {
    match params.get_by_id(id) {
        ParamValue::Float(value) => <R as FlightFloat>::from_f32(value),
        _ => <R as FlightFloat>::from_f32(0.0),
    }
}

fn param_int(params: &Params, id: ParamId) -> i32 {
    match params.get_by_id(id) {
        ParamValue::Int(value) => value,
        _ => 0,
    }
}

fn param_in_range(id: ParamId, first: ParamId, last: ParamId) -> bool {
    let id = id as usize;
    id >= first as usize && id <= last as usize
}

fn is_mixer_config_param(id: ParamId) -> bool {
    matches!(
        id,
        ParamId::PARAM_PRIMARY_MIXER | ParamId::PARAM_SECONDARY_MIXER
    ) || param_in_range(
        id,
        ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0,
        ParamId::PARAM_PRIMARY_MIXER_OUTPUT_9,
    ) || param_in_range(
        id,
        ParamId::PARAM_PRIMARY_MIXER_PWM_RATE_0,
        ParamId::PARAM_PRIMARY_MIXER_PWM_RATE_9,
    ) || param_in_range(
        id,
        ParamId::PARAM_PRIMARY_MIXER_0_0,
        ParamId::PARAM_PRIMARY_MIXER_9_9,
    ) || param_in_range(
        id,
        ParamId::PARAM_SECONDARY_MIXER_0_0,
        ParamId::PARAM_SECONDARY_MIXER_9_9,
    )
}

fn is_mixer_runtime_param(id: ParamId) -> bool {
    matches!(
        id,
        ParamId::PARAM_MOTOR_IDLE_THROTTLE
            | ParamId::PARAM_NUM_MOTORS
            | ParamId::PARAM_FIXED_WING
            | ParamId::PARAM_USE_MOTOR_PARAMETERS
            | ParamId::PARAM_RC_F_AXIS
    )
}

fn mixer_from_params<R: FlightFloat>(
    params: &Params,
    mixer_choice_id: ParamId,
    first_matrix_id: ParamId,
) -> Option<[[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS]> {
    if param_int(params, mixer_choice_id) != CUSTOM_MIXER {
        return None;
    }

    let mut mixer = [[<R as FlightFloat>::from_f32(0.0); NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS];
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

fn pwm_rates_from_params<R: FlightFloat>(params: &Params) -> Option<[R; NUM_MIXER_OUTPUTS]> {
    if param_int(params, ParamId::PARAM_PRIMARY_MIXER) != CUSTOM_MIXER {
        return None;
    }

    let mut rates = [<R as FlightFloat>::from_f32(0.0); NUM_MIXER_OUTPUTS];
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
            let Some(config) = canned_mixer::<f64>(choice) else {
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
            let Some(config) = canned_secondary_mixer::<f64>(choice) else {
                return;
            };
            save_secondary_mixer_params(params, config.matrix);
        }
        _ => {}
    }
}

fn save_primary_mixer_params(params: &mut Params, config: CannedMixerConfig<f64>) {
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
                ParamValue::Float(config.default_pwm_rates[output].to_f32_lossy()),
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
            params.set_by_id(param_id, ParamValue::Float(mixer_row[col].to_f32_lossy()));
        }
    }
}

fn matrix_from_default<R: FlightFloat>(
    matrix: [[f64; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
) -> [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] {
    matrix.map(|row| row.map(<R as FlightFloat>::from_f64))
}

fn rates_from_default<R: FlightFloat>(rates: [f64; NUM_MIXER_OUTPUTS]) -> [R; NUM_MIXER_OUTPUTS] {
    rates.map(<R as FlightFloat>::from_f64)
}

fn inverted_multirotor_mixer<R: FlightFloat>(
    mixer: [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
) -> [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] {
    let mut mixer_matrix = SMatrix::<R, NUM_MIXER_OUTPUTS, NUM_MIXER_OUTPUTS>::zeros();
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

    let mut sigma_inverse = SMatrix::<R, NUM_MIXER_OUTPUTS, NUM_MIXER_OUTPUTS>::zeros();
    for i in 0..NUM_MIXER_OUTPUTS {
        let singular_value = svd.singular_values[i];
        if singular_value != <R as FlightFloat>::from_f32(0.0) {
            sigma_inverse[(i, i)] = <R as FlightFloat>::from_f32(1.0) / singular_value;
        }
    }

    let mixer_matrix_pinv = v_t.transpose() * sigma_inverse * u.transpose();
    let mut inverted = [[<R as FlightFloat>::from_f32(0.0); NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS];
    for i in 0..NUM_MIXER_OUTPUTS {
        for j in 0..NUM_MIXER_OUTPUTS {
            inverted[j][i] = mixer_matrix_pinv[(i, j)];
        }
    }
    inverted
}

fn rank_one_pseudoinverse_mixer<R: FlightFloat>(
    mixer: [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS],
) -> [[R; NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS] {
    let mut norm_squared = <R as FlightFloat>::from_f32(0.0);
    for row in mixer.iter() {
        for value in row.iter() {
            norm_squared += *value * *value;
        }
    }

    if norm_squared < <R as FlightFloat>::from_f64(1.0e-12) {
        return mixer;
    }

    let mut inverted = [[<R as FlightFloat>::from_f32(0.0); NUM_MIXER_OUTPUTS]; NUM_MIXER_OUTPUTS];
    for row in 0..NUM_MIXER_OUTPUTS {
        for col in 0..NUM_MIXER_OUTPUTS {
            inverted[row][col] = mixer[row][col] / norm_squared;
        }
    }

    inverted
}

fn canned_mixer<R: FlightFloat>(choice: i32) -> Option<CannedMixerConfig<R>> {
    let config = match choice {
        ESC_CALIBRATION_MIXER => CannedMixerConfig {
            output_types: ALL_MOTOR_OUTPUT_TYPES,
            default_pwm_rates: rates_from_default(ESC_CALIBRATION_PWM_RATES),
            matrix: rank_one_pseudoinverse_mixer(matrix_from_default(ESC_CALIBRATION_MATRIX)),
        },
        QUAD_PLUS_MIXER => CannedMixerConfig {
            output_types: QUAD_X_OUTPUT_TYPES,
            default_pwm_rates: rates_from_default(QUAD_X_PWM_RATES),
            matrix: inverted_multirotor_mixer(matrix_from_default(QUAD_PLUS_MATRIX)),
        },
        QUAD_X_MIXER_CHOICE => CannedMixerConfig {
            output_types: QUAD_X_OUTPUT_TYPES,
            default_pwm_rates: rates_from_default(QUAD_X_PWM_RATES),
            matrix: inverted_multirotor_mixer(matrix_from_default(QUAD_X_MIXER)),
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
            default_pwm_rates: rates_from_default(HEX_PWM_RATES),
            matrix: inverted_multirotor_mixer(matrix_from_default(HEX_PLUS_MATRIX)),
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
            default_pwm_rates: rates_from_default(HEX_PWM_RATES),
            matrix: inverted_multirotor_mixer(matrix_from_default(HEX_X_MATRIX)),
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
            default_pwm_rates: rates_from_default(HEX_PWM_RATES),
            matrix: inverted_multirotor_mixer(matrix_from_default(Y6_MATRIX)),
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
            default_pwm_rates: rates_from_default(OCTO_PWM_RATES),
            matrix: inverted_multirotor_mixer(matrix_from_default(OCTO_PLUS_MATRIX)),
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
            default_pwm_rates: rates_from_default(OCTO_PWM_RATES),
            matrix: inverted_multirotor_mixer(matrix_from_default(OCTO_X_MATRIX)),
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
            default_pwm_rates: rates_from_default(OCTO_PWM_RATES),
            matrix: inverted_multirotor_mixer(matrix_from_default(X8_MATRIX)),
        },
        FIXEDWING_MIXER => CannedMixerConfig {
            output_types: FIXEDWING_OUTPUT_TYPES,
            default_pwm_rates: rates_from_default(FIXEDWING_PWM_RATES),
            matrix: matrix_from_default(FIXEDWING_MATRIX),
        },
        INVERTED_VTAIL_MIXER => CannedMixerConfig {
            output_types: FIXEDWING_OUTPUT_TYPES,
            default_pwm_rates: rates_from_default(FIXEDWING_PWM_RATES),
            matrix: matrix_from_default(INVERTED_VTAIL_MATRIX),
        },
        _ => return None,
    };

    Some(config)
}

fn canned_secondary_mixer<R: FlightFloat>(choice: i32) -> Option<CannedMixerConfig<R>> {
    let mut config = canned_mixer(choice)?;
    config.matrix = match choice {
        CUSTOM_MIXER => config.matrix,
        _ => inverted_multirotor_mixer(matrix_from_default(raw_canned_mixer_matrix(choice)?)),
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

fn apply_fixedwing_reversals<R: FlightFloat>(commands: &mut ControllerOutput<R>, params: &Params) {
    if param_int(params, ParamId::PARAM_AILERON_REVERSE) != 0 {
        commands.u[3] *= <R as FlightFloat>::from_f32(-1.0);
    }
    if param_int(params, ParamId::PARAM_ELEVATOR_REVERSE) != 0 {
        commands.u[4] *= <R as FlightFloat>::from_f32(-1.0);
    }
    if param_int(params, ParamId::PARAM_RUDDER_REVERSE) != 0 {
        commands.u[5] *= <R as FlightFloat>::from_f32(-1.0);
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
    ) -> MixerCtx<'a, f64> {
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
        let mut mixer = MatrixMixer::<f64>::new(&params);
        let mut controls = ControllerOutput::<f64>::default();
        controls.u[0] = 0.7;
        controls.u[3] = 0.1;
        controls.u[4] = -0.2;
        controls.u[5] = 0.3;

        let run = mixer.mix(&controls, mixer_ctx(&state, &params, OVERRIDE_NO_OVERRIDE));

        assert_eq!(run.status, MixerStatus::Healthy);
        assert_eq!(
            <MatrixMixer<f64> as Mixer<f64>>::output_types(&mixer),
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
        assert_eq!(
            <MatrixMixer<f64> as Mixer<f64>>::default_pwm_rates(&mixer),
            [50.0; NUM_MIXER_OUTPUTS]
        );
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
        let mut mixer = MatrixMixer::<f64>::new(&params);
        let mut controls = ControllerOutput::<f64>::default();
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
        let mut mixer = MatrixMixer::<f64>::new(&params);
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

    #[test]
    fn custom_motor_mixer_treats_negative_ned_fz_as_positive_thrust() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(CUSTOM_MIXER));
        params.set_by_id(ParamId::PARAM_SECONDARY_MIXER, ParamValue::Int(-1));
        params.set_by_id(ParamId::PARAM_USE_MOTOR_PARAMETERS, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_NUM_MOTORS, ParamValue::Int(4));
        params.set_by_id(ParamId::PARAM_MOTOR_RESISTANCE, ParamValue::Float(0.085));
        params.set_by_id(ParamId::PARAM_MOTOR_KV, ParamValue::Float(0.02894));
        params.set_by_id(ParamId::PARAM_NO_LOAD_CURRENT, ParamValue::Float(1.01));
        params.set_by_id(ParamId::PARAM_PROP_DIAMETER, ParamValue::Float(0.381));
        params.set_by_id(ParamId::PARAM_PROP_CQ, ParamValue::Float(0.0045));

        for output in 0..4 {
            params.set_by_id(
                ParamId::from_index(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0 as usize + output)
                    .unwrap(),
                ParamValue::Int(2),
            );
            params.set_by_id(
                ParamId::from_index(ParamId::PARAM_PRIMARY_MIXER_PWM_RATE_0 as usize + output)
                    .unwrap(),
                ParamValue::Float(490.0),
            );
            params.set_by_id(
                ParamId::from_index(
                    ParamId::PARAM_PRIMARY_MIXER_0_0 as usize + output * NUM_MIXER_OUTPUTS + 2,
                )
                .unwrap(),
                ParamValue::Float(-5814.7935),
            );
        }

        let state = StateManager::new();
        let mut mixer = MatrixMixer::<f64>::new(&params);
        let mut controls = ControllerOutput::<f64>::default();
        controls.u[2] = -25.0;

        let run = mixer.mix(
            &controls,
            MixerCtx {
                battery_voltage: Some(23.5),
                ..mixer_ctx(&state, &params, OVERRIDE_NO_OVERRIDE)
            },
        );

        assert_eq!(run.status, MixerStatus::Healthy);
        for output in 0..4 {
            assert!(
                run.commands[output] > 0.4 && run.commands[output] < 0.6,
                "motor {output} output was {}",
                run.commands[output]
            );
        }
    }
}
