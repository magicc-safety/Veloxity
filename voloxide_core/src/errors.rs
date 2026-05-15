pub enum TelemError {
    GenericTelemError(&'static str),
}

impl Default for TelemError {
    fn default() -> Self {
        TelemError::GenericTelemError("Default TelemError")
    }
}

#[derive(Debug, Clone, Copy)]
pub enum EstimatorError {
    GenericEstimatorError(&'static str),
}

impl Default for EstimatorError {
    fn default() -> Self {
        EstimatorError::GenericEstimatorError("Default EstimatorError")
    }
}

// #[derive(Debug, Clone, Copy, Format)]
#[derive(Debug, Clone, Copy)]
pub enum SensorError {
    GenericSensorError(&'static str),
}

impl Default for SensorError {
    fn default() -> Self {
        SensorError::GenericSensorError("Default SensorError")
    }
}
