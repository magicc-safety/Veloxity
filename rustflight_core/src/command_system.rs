use crate::{
    comm_messages::enums::RosflightCmd,
    command_manager::CommandManager,
    events::{CalibrationRequested, OffboardControlRequested, ParamDefaultsRequested},
    params2::Params,
    ports::EventDrainPort,
    sensorprocessors::CalibrationFlags,
};

pub struct CalibrationRequestCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, CalibrationRequested, N>,
    pub flags: &'a mut CalibrationFlags,
}

pub fn apply_calibration_requests<const N: usize>(mut ctx: CalibrationRequestCtx<'_, N>) {
    while let Some(request) = ctx.requests.next() {
        match request.command {
            RosflightCmd::AccelCalibration => ctx.flags.insert(CalibrationFlags::ACCEL),
            RosflightCmd::GyroCalibration => ctx.flags.insert(CalibrationFlags::GYRO),
            RosflightCmd::BaroCalibration => ctx.flags.insert(CalibrationFlags::BARO),
            RosflightCmd::AirspeedCalibration => ctx.flags.insert(CalibrationFlags::PITOT),
            _ => {}
        }
    }
}

pub struct OffboardControlCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, OffboardControlRequested, N>,
    pub command: &'a mut CommandManager,
    pub params: &'a Params,
}

pub fn apply_offboard_control_requests<const N: usize>(mut ctx: OffboardControlCtx<'_, N>) {
    while let Some(request) = ctx.requests.next() {
        ctx.command
            .set_new_offboard_command(request.now_us, &request.msg, ctx.params);
    }
}

pub struct ParamDefaultsCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, ParamDefaultsRequested, N>,
    pub params: &'a mut Params,
}

pub fn apply_param_defaults_requests<const N: usize>(
    mut ctx: ParamDefaultsCtx<'_, N>,
) -> Option<RosflightCmd> {
    let mut applied = None;
    while let Some(request) = ctx.requests.next() {
        ctx.params.set_defaults();
        applied = Some(request.command);
    }
    applied
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        comm_messages::{
            enums::{OffboardControlIgnore, OffboardControlMode},
            messages::OffboardControlMsg,
        },
        events::{
            CALIBRATION_REQUEST_QUEUE_CAPACITY, EventQueue, OFFBOARD_CONTROL_REQUEST_QUEUE_CAPACITY,
            PARAM_DEFAULTS_REQUEST_QUEUE_CAPACITY,
        },
        params2::{ParamId, ParamValue},
    };

    #[test]
    fn apply_calibration_requests_sets_requested_flags() {
        let mut requests =
            EventQueue::<CalibrationRequested, CALIBRATION_REQUEST_QUEUE_CAPACITY>::new();
        let mut flags = CalibrationFlags::empty();

        let _ = requests.push(CalibrationRequested {
            command: RosflightCmd::GyroCalibration,
        });
        let _ = requests.push(CalibrationRequested {
            command: RosflightCmd::BaroCalibration,
        });

        apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut requests),
            flags: &mut flags,
        });

        assert!(flags.contains(CalibrationFlags::GYRO));
        assert!(flags.contains(CalibrationFlags::BARO));
        assert!(requests.is_empty());
    }

    #[test]
    fn apply_offboard_control_requests_updates_command_manager() {
        let params = Params::new();
        let mut command = CommandManager::new();
        let mut requests =
            EventQueue::<OffboardControlRequested, OFFBOARD_CONTROL_REQUEST_QUEUE_CAPACITY>::new();

        let _ = requests.push(OffboardControlRequested {
            now_us: 42_000,
            msg: OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::IGNORE_QY,
                qx: 0.1,
                qy: 0.2,
                qz: 0.3,
                fx: 0.4,
                fy: 0.5,
                fz: 0.6,
            },
        });

        apply_offboard_control_requests(OffboardControlCtx {
            requests: EventDrainPort::new(&mut requests),
            command: &mut command,
            params: &params,
        });

        assert!(command.is_offboard_active());
        assert!(requests.is_empty());
    }

    #[test]
    fn apply_param_defaults_requests_resets_params_and_reports_command() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut requests =
            EventQueue::<ParamDefaultsRequested, PARAM_DEFAULTS_REQUEST_QUEUE_CAPACITY>::new();

        let _ = requests.push(ParamDefaultsRequested {
            command: RosflightCmd::SetParamDefaults,
        });

        let applied = apply_param_defaults_requests(ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut requests),
            params: &mut params,
        });

        assert!(matches!(applied, Some(RosflightCmd::SetParamDefaults)));
        assert_eq!(params.get_by_id(ParamId::PARAM_SYSTEM_ID), ParamValue::Int(1));
        assert!(requests.is_empty());
    }
}
