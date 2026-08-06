//! Battery-monitor ADC conversion shared by hardware board adapters.

/// Full-scale count produced by a 16-bit ADC conversion.
pub const ADC_16_BIT_FULL_SCALE: f32 = u16::MAX as f32;

/// Scale and offset for one analog battery-monitor channel.
///
/// This follows ROSflight C's conversion order exactly:
/// `(pin_voltage - offset) * scale_factor`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalogChannelCalibration {
    pub scale_factor: f32,
    pub offset: f32,
}

impl AnalogChannelCalibration {
    pub const fn new(scale_factor: f32, offset: f32) -> Self {
        Self {
            scale_factor,
            offset,
        }
    }

    /// Apply ROSflight's multiplier semantics. Zero means "leave the current
    /// board scale unchanged"; every nonzero value replaces it.
    pub fn apply_multiplier(&mut self, multiplier: f32) {
        if multiplier != 0.0 {
            self.scale_factor = multiplier;
        }
    }

    pub fn convert_16_bit_sample(self, adc_count: u16, vdda: f32) -> f32 {
        let pin_voltage = f32::from(adc_count) / ADC_16_BIT_FULL_SCALE * vdda;
        (pin_voltage - self.offset) * self.scale_factor
    }
}

/// Calibration for the two channels exposed by a standard power monitor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryMonitorCalibration {
    pub voltage: AnalogChannelCalibration,
    pub current: AnalogChannelCalibration,
}

impl BatteryMonitorCalibration {
    pub const fn new(
        voltage_scale: f32,
        voltage_offset: f32,
        current_scale: f32,
        current_offset: f32,
    ) -> Self {
        Self {
            voltage: AnalogChannelCalibration::new(voltage_scale, voltage_offset),
            current: AnalogChannelCalibration::new(current_scale, current_offset),
        }
    }

    pub fn apply_multipliers(&mut self, voltage_multiplier: f32, current_multiplier: f32) {
        self.voltage.apply_multiplier(voltage_multiplier);
        self.current.apply_multiplier(current_multiplier);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversion_matches_rosflight_order_of_operations() {
        let calibration = AnalogChannelCalibration::new(60.5, 0.0747);
        let value = calibration.convert_16_bit_sample(32_768, 3.3);
        let expected = (f32::from(32_768_u16) / 65_535.0 * 3.3 - 0.0747) * 60.5;
        assert!((value - expected).abs() < 1.0e-5);
    }

    #[test]
    fn nonzero_multiplier_replaces_board_scale() {
        let mut calibration = BatteryMonitorCalibration::new(12.62, 0.0, 60.5, 0.0747);
        calibration.apply_multipliers(7.675, 42.0);
        assert_eq!(calibration.voltage.scale_factor, 7.675);
        assert_eq!(calibration.current.scale_factor, 42.0);
    }

    #[test]
    fn zero_multiplier_leaves_current_scale_unchanged() {
        let mut calibration = BatteryMonitorCalibration::new(12.62, 0.0, 60.5, 0.0747);
        calibration.apply_multipliers(7.675, 0.0);
        calibration.apply_multipliers(0.0, 0.0);
        assert_eq!(calibration.voltage.scale_factor, 7.675);
        assert_eq!(calibration.current.scale_factor, 60.5);
    }
}
