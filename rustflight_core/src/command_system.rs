use crate::{
    comm_messages::enums::RosflightCmd,
    events::CalibrationRequested,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events::{CALIBRATION_REQUEST_QUEUE_CAPACITY, EventQueue};

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
}
