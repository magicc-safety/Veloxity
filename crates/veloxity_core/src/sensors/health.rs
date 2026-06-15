use crate::{
    math::FlightFloat,
    params::{ParamId, ParamValue, Params},
    sensors::ProcessedSensors,
    state_machine::{ErrorFlag, StateManager},
};

pub struct SensorHealthCtx<'a, R: FlightFloat> {
    pub now_us: u64,
    pub sensors: &'a ProcessedSensors<R>,
    pub params: &'a Params,
    pub state: &'a mut StateManager,
    pub last_imu_seen: &'a mut u64,
    pub imu_timeout_us: u64,
}

pub fn update_sensor_health<R: FlightFloat>(ctx: SensorHealthCtx<'_, R>) {
    if ctx.sensors.imu.is_some() {
        *ctx.last_imu_seen = ctx.now_us;
        ctx.state
            .set_error_flag(ErrorFlag::IMU_NOT_RESPONDING, false, ctx.params);
        update_imu_calibration_error(ctx.state, ctx.params);
    } else if ctx.now_us > *ctx.last_imu_seen + ctx.imu_timeout_us {
        ctx.state
            .set_error_flag(ErrorFlag::IMU_NOT_RESPONDING, true, ctx.params);
    }
}

fn update_imu_calibration_error(state: &mut StateManager, params: &Params) {
    let error = ErrorFlag::UNCALIBRATED_IMU;
    if imu_bias_params_are_all_zero(params) {
        state.set_error_flag(error, true, params);
    } else {
        state.set_error_flag(error, false, params);
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
    use crate::{
        packets::{ImuPacket, RosflightPacketHeader},
        state_machine::Event,
    };

    const TEST_IMU_TIMEOUT_US: u64 = 100_000;

    fn imu_packet(timestamp: u64) -> ImuPacket<f64> {
        ImuPacket {
            header: RosflightPacketHeader {
                timestamp,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        }
    }

    #[test]
    fn imu_calibration_health_sets_uncalibrated_error_when_all_bias_params_are_zero() {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        let mut last_imu_seen = 0;
        let sensors = ProcessedSensors {
            imu: Some(imu_packet(1)),
            ..ProcessedSensors::<f64>::default()
        };

        update_sensor_health(SensorHealthCtx {
            now_us: 1,
            sensors: &sensors,
            params: &params,
            state: &mut state,
            last_imu_seen: &mut last_imu_seen,
            imu_timeout_us: TEST_IMU_TIMEOUT_US,
        });

        assert!(state.get_errors().contains(ErrorFlag::UNCALIBRATED_IMU));
        assert_eq!(last_imu_seen, 1);
    }

    #[test]
    fn imu_calibration_health_clears_uncalibrated_error_when_any_bias_param_is_nonzero() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_ACC_X_BIAS, ParamValue::Float(0.01));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update(Event::ERROR_OCCURRED(ErrorFlag::UNCALIBRATED_IMU), &params);
        let mut last_imu_seen = 0;
        let sensors = ProcessedSensors {
            imu: Some(imu_packet(1)),
            ..ProcessedSensors::<f64>::default()
        };

        update_sensor_health(SensorHealthCtx {
            now_us: 1,
            sensors: &sensors,
            params: &params,
            state: &mut state,
            last_imu_seen: &mut last_imu_seen,
            imu_timeout_us: TEST_IMU_TIMEOUT_US,
        });

        assert!(!state.get_errors().contains(ErrorFlag::UNCALIBRATED_IMU));
    }

    #[test]
    fn sensor_health_sets_imu_not_responding_after_timeout_without_imu_sample() {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        let mut last_imu_seen = 0;
        let sensors = ProcessedSensors::<f64>::default();

        update_sensor_health(SensorHealthCtx {
            now_us: TEST_IMU_TIMEOUT_US + 1,
            sensors: &sensors,
            params: &params,
            state: &mut state,
            last_imu_seen: &mut last_imu_seen,
            imu_timeout_us: TEST_IMU_TIMEOUT_US,
        });

        assert!(state.get_errors().contains(ErrorFlag::IMU_NOT_RESPONDING));
        assert_eq!(last_imu_seen, 0);
    }

    #[test]
    fn sensor_health_clears_imu_not_responding_when_imu_sample_returns() {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update(
            Event::ERROR_OCCURRED(ErrorFlag::IMU_NOT_RESPONDING),
            &params,
        );
        let mut last_imu_seen = 0;
        let sensors = ProcessedSensors {
            imu: Some(imu_packet(5)),
            ..ProcessedSensors::<f64>::default()
        };

        update_sensor_health(SensorHealthCtx {
            now_us: 5,
            sensors: &sensors,
            params: &params,
            state: &mut state,
            last_imu_seen: &mut last_imu_seen,
            imu_timeout_us: TEST_IMU_TIMEOUT_US,
        });

        assert!(!state.get_errors().contains(ErrorFlag::IMU_NOT_RESPONDING));
        assert_eq!(last_imu_seen, 5);
    }
}
