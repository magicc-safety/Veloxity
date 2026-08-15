//! Pixracer Pro analog battery monitor.
//!
//! The board routes battery voltage and current from its power-monitor input to
//! ADC1 channels 14 (PA2) and 15 (PA3). ADC3's internal VREFINT channel is read
//! alongside them so conversion uses measured VDDA, matching ROSflight C.

use stm_32::embassy_stm32::{
    Peri,
    adc::{Adc, AdcChannel, AdcConfig, AnyAdcChannel, Resolution, SampleTime},
    pac,
    peripherals::{ADC1, ADC3, DMA2_CH0, DMA2_CH1, PA2, PA3},
};
use stm_32::embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use stm_32::embassy_time::{Duration, Instant, Timer, block_for, with_timeout};
use veloxity_core::{
    battery::BatteryMonitorCalibration,
    errors::SensorError,
    packets::{BatteryPacket, RosflightPacketHeader},
};

#[cfg(feature = "runtime-diagnostics")]
use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_BATTERY_SAMPLE_COUNT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_BATTERY_SAMPLE_SUM_US: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_BATTERY_SAMPLE_MAX_US: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_BATTERY_PUBLISH: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_BATTERY_ERROR_PUBLISH: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_BATTERY_SIGNAL_OVERWRITE: AtomicU32 = AtomicU32::new(0);

const SAMPLE_PERIOD: Duration = Duration::from_millis(100);
const ADC_CONVERSION_TIMEOUT: Duration = Duration::from_millis(5);
const ADC_SAMPLE_TIME: SampleTime = SampleTime::CYCLES810_5;

// Values from the ROSflight Pixracer Pro BoardConfig.h ADC channel table.
const BOARD_CALIBRATION: BatteryMonitorCalibration =
    BatteryMonitorCalibration::new(12.62, 0.0, 60.5, 0.0747);

// STM32H743/H753 factory VREFINT calibration: 16-bit ADC count measured with
// VDDA = 3.3 V. This is the address and reference used by the STM32 HAL and the
// ROSflight C firmware for this board.
const VREFINT_CAL_ADDR: *const u16 = 0x1FF1_E860 as *const u16;
const VREFINT_CAL_VDDA: f32 = 3.3;
#[derive(Debug, Clone, Copy)]
pub struct BatteryMultipliers {
    pub voltage: f32,
    pub current: f32,
}

pub static BATTERY_SIGNAL: Signal<CriticalSectionRawMutex, Result<BatteryPacket, SensorError>> =
    Signal::new();

static MULTIPLIER_SIGNAL: Signal<CriticalSectionRawMutex, BatteryMultipliers> = Signal::new();

pub fn configure_multipliers(voltage: f32, current: f32) {
    MULTIPLIER_SIGNAL.signal(BatteryMultipliers { voltage, current });
}

pub struct PixracerBatteryMonitor {
    adc1: Adc<'static, ADC1>,
    adc3: Adc<'static, ADC3>,
    adc1_dma: Peri<'static, DMA2_CH0>,
    adc3_dma: Peri<'static, DMA2_CH1>,
    voltage_channel: AnyAdcChannel<'static, ADC1>,
    current_channel: AnyAdcChannel<'static, ADC1>,
    vref_channel: AnyAdcChannel<'static, ADC3>,
    calibration: BatteryMonitorCalibration,
}

impl PixracerBatteryMonitor {
    pub fn new(
        adc1: Peri<'static, ADC1>,
        adc3: Peri<'static, ADC3>,
        voltage_pin: Peri<'static, PA2>,
        current_pin: Peri<'static, PA3>,
        adc1_dma: Peri<'static, DMA2_CH0>,
        adc3_dma: Peri<'static, DMA2_CH1>,
    ) -> Self {
        let config = AdcConfig {
            resolution: Some(Resolution::BITS16),
            ..Default::default()
        };
        let adc1 = Adc::new_with_config(adc1, config);
        let adc3 = Adc::new_with_config(
            adc3,
            AdcConfig {
                resolution: Some(Resolution::BITS16),
                ..Default::default()
            },
        );

        configure_adc_hardware();
        let voltage_channel: AnyAdcChannel<'static, ADC1> = voltage_pin.degrade_adc();
        let current_channel: AnyAdcChannel<'static, ADC1> = current_pin.degrade_adc();
        // Embassy's STM32H7 ADC v4 mapping correctly selects ADC3 channel 19.
        // Keep VREFINT on ADC3, as required by the STM32H753/Pixracer Pro.
        let vref_channel: AnyAdcChannel<'static, ADC3> = adc3.enable_vrefint().degrade_adc();

        Self {
            adc1,
            adc3,
            adc1_dma,
            adc3_dma,
            voltage_channel,
            current_channel,
            vref_channel,
            calibration: BOARD_CALIBRATION,
        }
    }

    async fn sample(&mut self) -> Result<BatteryPacket, SensorError> {
        if let Some(multipliers) = MULTIPLIER_SIGNAL.try_take() {
            self.calibration
                .apply_multipliers(multipliers.voltage, multipliers.current);
        }

        let started_us = Instant::now().as_micros();
        let mut vref_reading = [0u16; 1];
        with_timeout(
            ADC_CONVERSION_TIMEOUT,
            self.adc3.read(
                self.adc3_dma.reborrow(),
                crate::board::BoardIrqs,
                [(&mut self.vref_channel, ADC_SAMPLE_TIME)].into_iter(),
                &mut vref_reading,
            ),
        )
        .await
        .map_err(|_| SensorError::GenericSensorError("ADC VREFINT DMA conversion timed out"))?;

        let mut battery_readings = [0u16; 2];
        with_timeout(
            ADC_CONVERSION_TIMEOUT,
            self.adc1.read(
                self.adc1_dma.reborrow(),
                crate::board::BoardIrqs,
                [
                    (&mut self.voltage_channel, ADC_SAMPLE_TIME),
                    (&mut self.current_channel, ADC_SAMPLE_TIME),
                ]
                .into_iter(),
                &mut battery_readings,
            ),
        )
        .await
        .map_err(|_| SensorError::GenericSensorError("ADC1 DMA conversion timed out"))?;
        let completed_us = Instant::now().as_micros();

        let vref_count = vref_reading[0];
        let voltage_count = battery_readings[0];
        let current_count = battery_readings[1];

        let vdda = measured_vdda(vref_count).ok_or(SensorError::GenericSensorError(
            "ADC VREFINT conversion failed",
        ))?;
        Ok(BatteryPacket {
            header: RosflightPacketHeader {
                timestamp: started_us + completed_us.saturating_sub(started_us) / 2,
                status: 0,
            },
            voltage: self
                .calibration
                .voltage
                .convert_16_bit_sample(voltage_count, vdda),
            current: self
                .calibration
                .current
                .convert_16_bit_sample(current_count, vdda),
        })
    }

    async fn run(&mut self) -> ! {
        loop {
            Timer::at(stm_32::synch_at(SAMPLE_PERIOD)).await;
            #[cfg(feature = "runtime-diagnostics")]
            let started = Instant::now();
            let sample = self.sample().await;
            #[cfg(feature = "runtime-diagnostics")]
            {
                let elapsed_us = started.elapsed().as_micros().min(u32::MAX as u64) as u32;
                VELOXITY_DIAG_BATTERY_SAMPLE_COUNT.fetch_add(1, Ordering::Relaxed);
                VELOXITY_DIAG_BATTERY_SAMPLE_SUM_US.fetch_add(elapsed_us, Ordering::Relaxed);
                VELOXITY_DIAG_BATTERY_SAMPLE_MAX_US.fetch_max(elapsed_us, Ordering::Relaxed);
                VELOXITY_DIAG_BATTERY_PUBLISH.fetch_add(1, Ordering::Relaxed);
                if sample.is_err() {
                    VELOXITY_DIAG_BATTERY_ERROR_PUBLISH.fetch_add(1, Ordering::Relaxed);
                }
                if BATTERY_SIGNAL.signaled() {
                    VELOXITY_DIAG_BATTERY_SIGNAL_OVERWRITE.fetch_add(1, Ordering::Relaxed);
                }
            }
            BATTERY_SIGNAL.signal(sample);
        }
    }
}

fn configure_adc_hardware() {
    use pac::adc::vals::Boost;
    use pac::adccommon::vals::Presc;

    // ROSflight C uses DIV32 for the Pixracer Pro external ADC. Use the same
    // conservative clock for ADC3 as well; conversion ratios are unaffected,
    // and VREFINT is sampled far below its maximum rate.
    pac::ADC12_COMMON
        .ccr()
        .modify(|register| register.set_presc(Presc::DIV32));
    pac::ADC3_COMMON.ccr().modify(|register| {
        register.set_presc(Presc::DIV32);
        register.set_vrefen(true);
    });
    pac::ADC1
        .cr()
        .modify(|register| register.set_boost(Boost::LT6_25));
    pac::ADC3
        .cr()
        .modify(|register| register.set_boost(Boost::LT6_25));

    // STM32H753 datasheet maximum VREFINT startup time is below 15 us.
    block_for(Duration::from_micros(15));
}

fn measured_vdda(vref_count: u16) -> Option<f32> {
    if vref_count == 0 {
        return None;
    }
    // SAFETY: this read-only system-memory address is defined by STM32H743/H753
    // and contains ST's factory-programmed VREFINT calibration value.
    let calibrated_count = unsafe { core::ptr::read_volatile(VREFINT_CAL_ADDR) };
    if calibrated_count == 0 {
        return None;
    }
    Some(VREFINT_CAL_VDDA * f32::from(calibrated_count) / f32::from(vref_count))
}

#[embassy_executor::task]
pub async fn task(mut monitor: PixracerBatteryMonitor) {
    monitor.run().await;
}
