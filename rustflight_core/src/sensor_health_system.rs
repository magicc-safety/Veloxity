use crate::{
    params::{ParamId, ParamValue, Params},
    state_machine::{ErrorFlag, Event, StateManager},
};

pub struct ImuCalibrationHealthCtx<'a> {
    pub params: &'a Params,
    pub state: &'a mut StateManager,
}

pub fn update_imu_calibration_error(ctx: ImuCalibrationHealthCtx<'_>) {
    let error = ErrorFlag::UNCALIBRATED_IMU;
    if imu_bias_params_are_all_zero(ctx.params) {
        ctx.state.update(Event::ERROR_OCCURRED(error), ctx.params);
    } else {
        ctx.state.update(Event::ERROR_CLEARED(error), ctx.params);
    }
}

fn imu_bias_params_are_all_zero(params: &Params) -> bool {
    [
        ParamId::PARAM_ACC_X_BIAS,
        ParamId::PARAM_ACC_Y_BIAS,
        ParamId::PARAM_ACC_Z_BIAS,
        ParamId::PARAM_GYRO_X_BIAS,
        ParamId::PARAM_GYRO_Y_BIAS,
        ParamId::PARAM_GYRO_Z_BIAS,
    ]
    .into_iter()
    .all(|id| matches!(params.get_by_id(id), ParamValue::Float(value) if value == 0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_calibration_health_sets_uncalibrated_error_when_all_bias_params_are_zero() {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);

        update_imu_calibration_error(ImuCalibrationHealthCtx {
            params: &params,
            state: &mut state,
        });

        assert!(state.get_errors().contains(ErrorFlag::UNCALIBRATED_IMU));
    }

    #[test]
    fn imu_calibration_health_clears_uncalibrated_error_when_any_bias_param_is_nonzero() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_ACC_X_BIAS, ParamValue::Float(0.01));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update(Event::ERROR_OCCURRED(ErrorFlag::UNCALIBRATED_IMU), &params);

        update_imu_calibration_error(ImuCalibrationHealthCtx {
            params: &params,
            state: &mut state,
        });

        assert!(!state.get_errors().contains(ErrorFlag::UNCALIBRATED_IMU));
    }
}
