//! Feature-gated sensor producer diagnostics.
//!
//! These counters observe the last-value `Signal` boundary used by board sensor
//! tasks. A publish while the signal is already occupied means the previous
//! result was replaced before the flight loop consumed it.

use core::sync::atomic::{AtomicU32, Ordering};

macro_rules! sensor_counters {
    ($publish:ident, $errors:ident, $overwrite:ident) => {
        #[unsafe(no_mangle)]
        pub static $publish: AtomicU32 = AtomicU32::new(0);
        #[unsafe(no_mangle)]
        pub static $errors: AtomicU32 = AtomicU32::new(0);
        #[unsafe(no_mangle)]
        pub static $overwrite: AtomicU32 = AtomicU32::new(0);
    };
}

sensor_counters!(
    VELOXITY_DIAG_MAG_PUBLISH,
    VELOXITY_DIAG_MAG_ERROR_PUBLISH,
    VELOXITY_DIAG_MAG_SIGNAL_OVERWRITE
);
sensor_counters!(
    VELOXITY_DIAG_BARO_PUBLISH,
    VELOXITY_DIAG_BARO_ERROR_PUBLISH,
    VELOXITY_DIAG_BARO_SIGNAL_OVERWRITE
);
sensor_counters!(
    VELOXITY_DIAG_PITOT_PUBLISH,
    VELOXITY_DIAG_PITOT_ERROR_PUBLISH,
    VELOXITY_DIAG_PITOT_SIGNAL_OVERWRITE
);
sensor_counters!(
    VELOXITY_DIAG_RANGE_PUBLISH,
    VELOXITY_DIAG_RANGE_ERROR_PUBLISH,
    VELOXITY_DIAG_RANGE_SIGNAL_OVERWRITE
);
sensor_counters!(
    VELOXITY_DIAG_GNSS_PUBLISH,
    VELOXITY_DIAG_GNSS_ERROR_PUBLISH,
    VELOXITY_DIAG_GNSS_SIGNAL_OVERWRITE
);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_MAG_CONVERSION_COMMAND: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_MAG_DRDY_READY: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_MAG_DRDY_MISS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_MAG_I2C_ERROR: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_RC_PUBLISH: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_RC_ERROR_PUBLISH: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_RC_QUEUE_FULL_WAITS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_RC_QUEUE_WAIT_SUM_US: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_RC_QUEUE_WAIT_MAX_US: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_RC_QUEUE_DEPTH_MAX: AtomicU32 = AtomicU32::new(0);
sensor_counters!(
    VELOXITY_DIAG_PPS_PUBLISH,
    VELOXITY_DIAG_PPS_ERROR_PUBLISH,
    VELOXITY_DIAG_PPS_SIGNAL_OVERWRITE
);

#[derive(Clone, Copy)]
pub enum SensorKind {
    Mag,
    Baro,
    Pitot,
    Range,
    Gnss,
    Pps,
}

pub fn record_signal_publish(kind: SensorKind, pending: bool, error: bool) {
    let (publish, errors, overwrite) = match kind {
        SensorKind::Mag => (
            &VELOXITY_DIAG_MAG_PUBLISH,
            &VELOXITY_DIAG_MAG_ERROR_PUBLISH,
            &VELOXITY_DIAG_MAG_SIGNAL_OVERWRITE,
        ),
        SensorKind::Baro => (
            &VELOXITY_DIAG_BARO_PUBLISH,
            &VELOXITY_DIAG_BARO_ERROR_PUBLISH,
            &VELOXITY_DIAG_BARO_SIGNAL_OVERWRITE,
        ),
        SensorKind::Pitot => (
            &VELOXITY_DIAG_PITOT_PUBLISH,
            &VELOXITY_DIAG_PITOT_ERROR_PUBLISH,
            &VELOXITY_DIAG_PITOT_SIGNAL_OVERWRITE,
        ),
        SensorKind::Range => (
            &VELOXITY_DIAG_RANGE_PUBLISH,
            &VELOXITY_DIAG_RANGE_ERROR_PUBLISH,
            &VELOXITY_DIAG_RANGE_SIGNAL_OVERWRITE,
        ),
        SensorKind::Gnss => (
            &VELOXITY_DIAG_GNSS_PUBLISH,
            &VELOXITY_DIAG_GNSS_ERROR_PUBLISH,
            &VELOXITY_DIAG_GNSS_SIGNAL_OVERWRITE,
        ),
        SensorKind::Pps => (
            &VELOXITY_DIAG_PPS_PUBLISH,
            &VELOXITY_DIAG_PPS_ERROR_PUBLISH,
            &VELOXITY_DIAG_PPS_SIGNAL_OVERWRITE,
        ),
    };
    publish.fetch_add(1, Ordering::Relaxed);
    if error {
        errors.fetch_add(1, Ordering::Relaxed);
    }
    if pending {
        overwrite.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn record_rc_queue_publish(error: bool, waited_us: Option<u32>, depth: usize) {
    VELOXITY_DIAG_RC_PUBLISH.fetch_add(1, Ordering::Relaxed);
    if error {
        VELOXITY_DIAG_RC_ERROR_PUBLISH.fetch_add(1, Ordering::Relaxed);
    }
    VELOXITY_DIAG_RC_QUEUE_DEPTH_MAX.fetch_max(depth as u32, Ordering::Relaxed);
    if let Some(waited_us) = waited_us {
        VELOXITY_DIAG_RC_QUEUE_FULL_WAITS.fetch_add(1, Ordering::Relaxed);
        VELOXITY_DIAG_RC_QUEUE_WAIT_SUM_US.fetch_add(waited_us, Ordering::Relaxed);
        VELOXITY_DIAG_RC_QUEUE_WAIT_MAX_US.fetch_max(waited_us, Ordering::Relaxed);
    }
}

pub fn record_mag_conversion_command() {
    VELOXITY_DIAG_MAG_CONVERSION_COMMAND.fetch_add(1, Ordering::Relaxed);
}

pub fn record_mag_drdy(ready: bool) {
    if ready {
        VELOXITY_DIAG_MAG_DRDY_READY.fetch_add(1, Ordering::Relaxed);
    } else {
        VELOXITY_DIAG_MAG_DRDY_MISS.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn record_mag_i2c_error() {
    VELOXITY_DIAG_MAG_I2C_ERROR.fetch_add(1, Ordering::Relaxed);
}
