use super::super::{
    BusOperation, DelayNs, Error, Ism330dhcx, RegisterOperation, SensorOperation, bisync,
    register::{BankState, MainBank},
};

use bitfield_struct::bitfield;
use derive_more::TryFrom;
use st_mem_bank_macro::{named_register, register};

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum Reg {
    /// Function configuration access register.
    ///
    /// Used to enable access to embedded functions and advanced features.
    FuncCfgAccess = 0x01,

    /// Pin control register.
    ///
    /// Configures the behavior of interrupt pins (INT1/INT2).
    PinCtrl = 0x02,

    /// FIFO control register 1.
    ///
    /// Configures the FIFO threshold level (low byte).
    FifoCtrl1 = 0x07,

    /// FIFO control register 2.
    ///
    /// Configures the FIFO threshold level (high byte) and mode.
    FifoCtrl2 = 0x08,

    /// FIFO control register 3.
    ///
    /// Configures the data sources for FIFO storage.
    FifoCtrl3 = 0x09,

    /// FIFO control register 4.
    ///
    /// Configures additional FIFO settings, such as decimation.
    FifoCtrl4 = 0x0A,

    /// Counter batch data rate register 1.
    ///
    /// Configures the batching data rate for the accelerometer.
    CounterBdrReg1 = 0x0B,

    /// Counter batch data rate register 2.
    ///
    /// Configures the batching data rate for the gyroscope.
    CounterBdrReg2 = 0x0C,

    /// Interrupt 1 control register.
    ///
    /// Configures the events routed to the INT1 pin.
    Int1Ctrl = 0x0D,

    /// Interrupt 2 control register.
    ///
    /// Configures the events routed to the INT2 pin.
    Int2Ctrl = 0x0E,

    /// Who Am I register.
    ///
    /// Read-only register containing the device ID. Default value: `0x6B`.
    WhoAmI = 0x0F,

    /// Accelerometer control register 1.
    ///
    /// Configures the accelerometer's output data rate and full-scale range.
    Ctrl1Xl = 0x10,

    /// Gyroscope control register 2.
    ///
    /// Configures the gyroscope's output data rate and full-scale range.
    Ctrl2G = 0x11,

    /// Control register 3.
    ///
    /// Configures general settings, such as boot, reset, and SPI mode.
    Ctrl3C = 0x12,

    /// Control register 4.
    ///
    /// Configures additional settings, such as data-ready interrupt enable.
    Ctrl4C = 0x13,

    /// Control register 5.
    ///
    /// Configures settings such as rounding and self-test.
    Ctrl5C = 0x14,

    /// Control register 6.
    ///
    /// Configures gyroscope settings, such as low-pass filter enable.
    Ctrl6C = 0x15,

    /// Control register 7.
    ///
    /// Configures gyroscope power mode and high-pass filter settings.
    Ctrl7G = 0x16,

    /// Accelerometer control register 8.
    ///
    /// Configures accelerometer filters and high-performance mode.
    Ctrl8Xl = 0x17,

    /// Accelerometer control register 9.
    ///
    /// Configures accelerometer data routing and power mode.
    Ctrl9Xl = 0x18,

    /// Control register 10.
    ///
    /// Configures embedded functions, such as pedometer and tilt detection.
    Ctrl10C = 0x19,

    /// All interrupt source register.
    ///
    /// Provides the status of all interrupt events.
    AllIntSrc = 0x1A,

    /// Wake-up source register.
    ///
    /// Indicates the source of wake-up events.
    WakeUpSrc = 0x1B,

    /// Tap source register.
    ///
    /// Indicates the source of tap detection events.
    TapSrc = 0x1C,

    /// 6D orientation source register.
    ///
    /// Indicates the source of 6D orientation events.
    D6dSrc = 0x1D,

    /// Status register or SPI auxiliary register.
    ///
    /// Provides the status of data-ready and FIFO events.
    StatusRegOrSpiaux = 0x1E,

    /// Temperature output register (low byte).
    OutTempL = 0x20,

    /// Temperature output register (high byte).
    OutTempH = 0x21,

    /// Gyroscope X-axis output register (low byte).
    OutxLG = 0x22,

    /// Gyroscope X-axis output register (high byte).
    OutxHG = 0x23,

    /// Gyroscope Y-axis output register (low byte).
    OutyLG = 0x24,

    /// Gyroscope Y-axis output register (high byte).
    OutyHG = 0x25,

    /// Gyroscope Z-axis output register (low byte).
    OutzLG = 0x26,

    /// Gyroscope Z-axis output register (high byte).
    OutzHG = 0x27,

    /// Accelerometer X-axis output register (low byte).
    OutxLA = 0x28,

    /// Accelerometer X-axis output register (high byte).
    OutxHA = 0x29,

    /// Accelerometer Y-axis output register (low byte).
    OutyLA = 0x2A,

    /// Accelerometer Y-axis output register (high byte).
    OutyHA = 0x2B,

    /// Accelerometer Z-axis output register (low byte).
    OutzLA = 0x2C,

    /// Accelerometer Z-axis output register (high byte).
    OutzHA = 0x2D,

    /// Embedded function status register (main page).
    ///
    /// Provides the status of embedded functions, such as FSM and MLC.
    EmbFuncStatusMainpage = 0x35,

    /// Finite state machine status register A (main page).
    ///
    /// Provides the status of FSM programs 1–8.
    FsmStatusAMainpage = 0x36,

    /// Finite state machine status register B (main page).
    ///
    /// Provides the status of FSM programs 9–16.
    FsmStatusBMainpage = 0x37,

    /// Machine learning core status register (main page).
    ///
    /// Provides the status of MLC events.
    MlcStatusMainpage = 0x38,

    /// Sensor hub status register (main page).
    ///
    /// Provides the status of sensor hub operations.
    StatusMasterMainpage = 0x39,

    /// FIFO status register 1.
    ///
    /// Provides the FIFO fill level (low byte).
    FifoStatus1 = 0x3A,

    /// FIFO status register 2.
    ///
    /// Provides the FIFO fill level (high byte) and status flags.
    FifoStatus2 = 0x3B,

    /// Timestamp output register 0.
    ///
    /// Provides the least significant byte of the timestamp.
    Timestamp0 = 0x40,

    /// Timestamp output register 1.
    ///
    /// Provides the middle byte of the timestamp.
    Timestamp1 = 0x41,

    /// Timestamp output register 2.
    ///
    /// Provides the most significant byte of the timestamp.
    Timestamp2 = 0x42,

    /// Timestamp output register 3.
    ///
    /// Provides an additional byte for extended timestamp resolution.
    Timestamp3 = 0x43,

    /// Tap configuration register 0.
    ///
    /// Configures tap detection settings, such as priority and thresholds.
    TapCfg0 = 0x56,

    /// Tap configuration register 1.
    ///
    /// Configures additional tap detection settings.
    TapCfg1 = 0x57,

    /// Tap configuration register 2.
    ///
    /// Configures Z-axis tap detection thresholds.
    TapCfg2 = 0x58,

    /// Tap threshold and 6D orientation configuration register.
    ///
    /// Configures thresholds for tap detection and 6D orientation.
    TapThs6d = 0x59,

    /// Interrupt duration register 2.
    ///
    /// Configures tap duration, quiet time, and shock time.
    IntDur2 = 0x5A,

    /// Wake-up threshold register.
    ///
    /// Configures the threshold for wake-up events.
    WakeUpThs = 0x5B,

    /// Wake-up duration register.
    ///
    /// Configures the duration for wake-up and sleep events.
    WakeUpDur = 0x5C,

    /// Free-fall configuration register.
    ///
    /// Configures thresholds and durations for free-fall detection.
    FreeFall = 0x5D,

    /// Interrupt configuration register 1.
    ///
    /// Configures events routed to the INT1 pin.
    Md1Cfg = 0x5E,

    /// Interrupt configuration register 2.
    ///
    /// Configures events routed to the INT2 pin.
    Md2Cfg = 0x5F,

    /// Internal fine frequency adjustment register.
    ///
    /// Configures the internal clock frequency.
    InternalFreqFine = 0x63,

    /// OIS interrupt configuration register.
    ///
    /// Configures OIS interrupt settings.
    IntOis = 0x6F,

    /// OIS control register 1.
    ///
    /// Configures OIS accelerometer settings.
    Ctrl1Ois = 0x70,

    /// OIS control register 2.
    ///
    /// Configures OIS gyroscope settings.
    Ctrl2Ois = 0x71,

    /// OIS control register 3.
    ///
    /// Configures additional OIS settings.
    Ctrl3Ois = 0x72,

    /// X-axis user offset register.
    ///
    /// Configures the X-axis offset for calibration.
    XOfsUsr = 0x73,

    /// Y-axis user offset register.
    ///
    /// Configures the Y-axis offset for calibration.
    YOfsUsr = 0x74,

    /// Z-axis user offset register.
    ///
    /// Configures the Z-axis offset for calibration.
    ZOfsUsr = 0x75,

    /// FIFO data output tag register.
    ///
    /// Identifies the sensor associated with the FIFO data.
    FifoDataOutTag = 0x78,

    /// FIFO data output register (X-axis low byte).
    FifoDataOutXL = 0x79,

    /// FIFO data output register (X-axis high byte).
    FifoDataOutXH = 0x7A,

    /// FIFO data output register (Y-axis low byte).
    FifoDataOutYL = 0x7B,

    /// FIFO data output register (Y-axis high byte).
    FifoDataOutYH = 0x7C,

    /// FIFO data output register (Z-axis low byte).
    FifoDataOutZL = 0x7D,

    /// FIFO data output register (Z-axis high byte).
    FifoDataOutZH = 0x7E,
}

/// Function Configuration Access Register (R/W).
///
/// The `FUNC_CFG_ACCESS` register is used to configure access to embedded functions and advanced features.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FuncCfgAccess, access_type = "Ism330dhcx<B, T, S>", multi_state = true)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FuncCfgAccess {
    #[bits(6, access = RO, default = 0)]
    not_used_01: u8,

    #[bits(2, default = 0)]
    pub reg_access: u8,
}

/// Pin Control Register (R/W).
///
/// The `PIN_CTRL` register is used to configure the pull-up and pull-down settings for specific pins.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::PinCtrl, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PinCtrl {
    #[bits(6, access = RO, default = 0b111111)]
    not_used_01: u8,

    #[bits(1, default = 0)]
    pub sdo_pu_en: u8,

    #[bits(1, default = 0)]
    pub ois_pu_dis: u8,
}

/// FIFO Control Register 1 (R/W).
///
/// The `FIFO_CTRL1` register is used to configure the FIFO watermark level.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FifoCtrl1, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl1 {
    #[bits(8, default = 0)]
    pub wtm: u8,
}

/// FIFO Control Register 2 (R/W).
///
/// The `FIFO_CTRL2` register is used to configure FIFO watermark, compression rate, and other FIFO-related settings.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FifoCtrl2, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl2 {
    #[bits(1, default = 0)]
    pub wtm: u8,

    #[bits(2, default = 0)]
    pub uncoptr_rate: u8,

    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,

    #[bits(1, default = 0)]
    pub odrchg_en: u8,

    #[bits(1, access = RO, default = 0)]
    not_used_02: u8,

    #[bits(1, default = 0)]
    pub fifo_compr_rt_en: u8,

    #[bits(1, default = 0)]
    pub stop_on_wtm: u8,
}

/// FIFO Control Register 3 (R/W).
///
/// The `FIFO_CTRL3` register is used to configure the batch data rate for the accelerometer and gyroscope.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FifoCtrl3, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl3 {
    #[bits(4, default = 0)]
    pub bdr_xl: u8,

    #[bits(4, default = 0)]
    pub bdr_gy: u8,
}

/// FIFO Control Register 4 (R/W).
///
/// The `FIFO_CTRL4` register is used to configure FIFO mode and batch data rates for temperature and timestamp.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FifoCtrl4, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl4 {
    #[bits(3, default = 0)]
    pub fifo_mode: u8,

    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,

    #[bits(2, default = 0)]
    pub odr_t_batch: u8,

    #[bits(2, default = 0)]
    pub odr_ts_batch: u8,
}

/// Counter BDR Register 1 and 2 (R/W).
///
/// The `COUNTER_BDR_REG1-2` register is used to configure the counter threshold and control related features.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::CounterBdrReg1, access_type = "Ism330dhcx<B, T, MainBank>", order = Inverse)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct CounterBdrReg {
    #[bits(11, default = 0)]
    pub cnt_bdr_th: u16,

    #[bits(2, access = RO, default = 0)]
    not_used_01: u8,

    #[bits(1, default = 0)]
    pub trig_counter_bdr: u8,

    #[bits(1, default = 0)]
    pub rst_counter_bdr: u8,

    #[bits(1, default = 0)]
    pub dataready_pulsed: u8,
}

/// Interrupt 1 Control Register (R/W).
///
/// The `INT1_CTRL` register is used to configure interrupt generation for various events on the INT1 pin.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Int1Ctrl, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Int1Ctrl {
    #[bits(1, default = 0)]
    pub int1_drdy_xl: u8,

    #[bits(1, default = 0)]
    pub int1_drdy_g: u8,

    #[bits(1, default = 0)]
    pub int1_boot: u8,

    #[bits(1, default = 0)]
    pub int1_fifo_th: u8,

    #[bits(1, default = 0)]
    pub int1_fifo_ovr: u8,

    #[bits(1, default = 0)]
    pub int1_fifo_full: u8,

    #[bits(1, default = 0)]
    pub int1_cnt_bdr: u8,

    #[bits(1, default = 0)]
    pub den_drdy_flag: u8,
}

/// Who Am I (R).
///
/// It's value is fixed at 0x6B. Contains the device id.
#[register(address = Reg::WhoAmI, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WhoAmI {
    #[bits(8, default = 0x6B)]
    pub id: u8,
}

/// Interrupt 2 Control Register (R/W).
///
/// The `INT2_CTRL` register is used to configure interrupt generation for various events on the INT2 pin.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Int2Ctrl, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Int2Ctrl {
    #[bits(1, default = 0)]
    pub int2_drdy_xl: u8,

    #[bits(1, default = 0)]
    pub int2_drdy_g: u8,

    #[bits(1, default = 0)]
    pub int2_drdy_temp: u8,

    #[bits(1, default = 0)]
    pub int2_fifo_th: u8,

    #[bits(1, default = 0)]
    pub int2_fifo_ovr: u8,

    #[bits(1, default = 0)]
    pub int2_fifo_full: u8,

    #[bits(1, default = 0)]
    pub int2_cnt_bdr: u8,

    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
}

/// Control Register 1 for Accelerometer (R/W).
///
/// The `CTRL1_XL` register is used to configure the accelerometer settings, including output data rate and full scale.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl1Xl, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl1Xl {
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,

    #[bits(1, default = 0)]
    pub lpf2_xl_en: u8,

    #[bits(2, default = 0)]
    pub fs_xl: u8,

    #[bits(4, default = 0)]
    pub odr_xl: u8,
}

/// Control Register 2 for Gyroscope (R/W).
///
/// The `CTRL2_G` register is used to configure the gyroscope settings, including output data rate and full scale.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl2G, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl2G {
    #[bits(4, default = 0)]
    pub fs_g: u8,

    #[bits(4, default = 0)]
    pub odr_g: u8,
}

/// Control Register 3 (R/W).
///
/// The `CTRL3_C` register is used to configure general control settings, including reset, boot, and data alignment.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl3C, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl3C {
    #[bits(1, default = 0)]
    pub sw_reset: u8,

    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,

    #[bits(1, default = 1)]
    pub if_inc: u8,

    #[bits(1, default = 0)]
    pub sim: u8,

    #[bits(1, default = 0)]
    pub pp_od: u8,

    #[bits(1, default = 0)]
    pub h_lactive: u8,

    #[bits(1, default = 0)]
    pub bdu: u8,

    #[bits(1, default = 0)]
    pub boot: u8,
}

/// Control register 4 (R/W).
///
/// The `CTRL4_C` register is used to configure various control settings for the device, such as enabling/disabling I2C, masking data-ready signals, and configuring gyroscope sleep mode.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl4C, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl4C {
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub lpf1_sel_g: u8,
    #[bits(1, default = 0)]
    pub i2c_disable: u8,
    #[bits(1, default = 0)]
    pub drdy_mask: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_02: u8,
    #[bits(1, default = 0)]
    pub int2_on_int1: u8,
    #[bits(1, default = 0)]
    pub sleep_g: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_03: u8,
}

/// Control register 5 (R/W).
///
/// The `CTRL5_C` register is used to configure self-test modes for the accelerometer and gyroscope, as well as rounding modes for data output.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl5C, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl5C {
    #[bits(2, default = 0)]
    pub st_xl: u8,
    #[bits(2, default = 0)]
    pub st_g: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(2, default = 0)]
    pub rounding: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_02: u8,
}

/// Control register 6 (R/W).
///
/// The `CTRL6_C` register is used to configure filtering options, user offset weight, accelerometer high-performance mode, and DEN mode.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl6C, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl6C {
    #[bits(3, default = 0)]
    pub ftype: u8,
    #[bits(1, default = 0)]
    pub usr_off_w: u8,
    #[bits(1, default = 0)]
    pub xl_hm_mode: u8,
    #[bits(3, default = 0)]
    pub den_mode: u8,
}

/// Control register 7 (R/W).
///
/// The `CTRL7_G` register is used to configure gyroscope high-pass filters, gyroscope high-performance mode, and OIS (optical image stabilization) settings.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl7G, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl7G {
    #[bits(1, default = 0)]
    pub ois_on: u8,
    #[bits(1, default = 0)]
    pub usr_off_on_out: u8,
    #[bits(1, default = 0)]
    pub ois_on_en: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(2, default = 0)]
    pub hpm_g: u8,
    #[bits(1, default = 0)]
    pub hp_en_g: u8,
    #[bits(1, default = 0)]
    pub g_hm_mode: u8,
}

/// Control register 8 (R/W).
///
/// The `CTRL8_XL` register is used to configure accelerometer filtering options, including high-pass filters and slope filters.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl8Xl, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl8Xl {
    #[bits(1, default = 0)]
    pub low_pass_on_6d: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub hp_slope_xl_en: u8,
    #[bits(1, default = 0)]
    pub fastsettl_mode_xl: u8,
    #[bits(1, default = 0)]
    pub hp_ref_mode_xl: u8,
    #[bits(3, default = 0)]
    pub hpcf_xl: u8,
}

/// Control register 9 (R/W).
///
/// The `CTRL9_XL` register is used to configure DEN functionality and device configuration.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl9Xl, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl9Xl {
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub device_conf: u8,
    #[bits(1, default = 0)]
    pub den_lh: u8,
    #[bits(2, default = 0)]
    pub den_xl_g: u8,
    #[bits(1, default = 1)]
    pub den_z: u8,
    #[bits(1, default = 1)]
    pub den_y: u8,
    #[bits(1, default = 1)]
    pub den_x: u8,
}

/// Control register 10 (R/W).
///
/// The `CTRL10_C` register is used to enable timestamp functionality.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl10C, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl10C {
    #[bits(5, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub timestamp_en: u8,
    #[bits(2, access = RO, default = 0)]
    not_used_02: u8,
}

/// All interrupt source register (R).
///
/// The `ALL_INT_SRC` register provides information about various interrupt events, such as free-fall detection, wake-up events, tap detection, and timestamp end count.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::AllIntSrc, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct AllIntSrc {
    #[bits(1, access = RO)]
    pub ff_ia: u8,
    #[bits(1, access = RO)]
    pub wu_ia: u8,
    #[bits(1, access = RO)]
    pub single_tap: u8,
    #[bits(1, access = RO)]
    pub double_tap: u8,
    #[bits(1, access = RO)]
    pub d6d_ia: u8,
    #[bits(1, access = RO)]
    pub sleep_change_ia: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, access = RO)]
    pub timestamp_endcount: u8,
}

/// Wake-up source register (R).
///
/// The `WAKE_UP_SRC` register provides information about wake-up events, free-fall detection, and sleep state changes.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::WakeUpSrc, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WakeUpSrc {
    #[bits(1, access = RO)]
    pub z_wu: u8,
    #[bits(1, access = RO)]
    pub y_wu: u8,
    #[bits(1, access = RO)]
    pub x_wu: u8,
    #[bits(1, access = RO)]
    pub wu_ia: u8,
    #[bits(1, access = RO)]
    pub sleep_state: u8,
    #[bits(1, access = RO)]
    pub ff_ia: u8,
    #[bits(1, access = RO)]
    pub sleep_change_ia: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
}

/// Tap source register (R).
///
/// The `TAP_SRC` register provides information about tap detection events, including single-tap, double-tap, and the axis of the detected tap.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::TapSrc, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapSrc {
    #[bits(1, access = RO)]
    pub z_tap: u8,
    #[bits(1, access = RO)]
    pub y_tap: u8,
    #[bits(1, access = RO)]
    pub x_tap: u8,
    #[bits(1, access = RO)]
    pub tap_sign: u8,
    #[bits(1, access = RO)]
    pub double_tap: u8,
    #[bits(1, access = RO)]
    pub single_tap: u8,
    #[bits(1, access = RO)]
    pub tap_ia: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
}

/// 6D orientation source register (R).
///
/// The `D6D_SRC` register provides information about the 6D orientation detection, including the direction of movement along the X, Y, and Z axes.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::D6dSrc, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct D6dSrc {
    #[bits(1, access = RO)]
    pub xl: u8,
    #[bits(1, access = RO)]
    pub xh: u8,
    #[bits(1, access = RO)]
    pub yl: u8,
    #[bits(1, access = RO)]
    pub yh: u8,
    #[bits(1, access = RO)]
    pub zl: u8,
    #[bits(1, access = RO)]
    pub zh: u8,
    #[bits(1, access = RO)]
    pub d6d_ia: u8,
    #[bits(1, access = RO)]
    pub den_drdy: u8,
}

#[register(address = Reg::OutTempL, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct OutTempReg {
    #[bits(16, access = RO)]
    pub temp: i16,
}

#[named_register(address = Reg::OutxLG, access_type = "Ism330dhcx<B, T, MainBank>")]
pub struct OutXYZG {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[named_register(address = Reg::OutxLA, access_type = "Ism330dhcx<B, T, MainBank>")]
pub struct OutXYZA {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Status register (R).
///
/// The `STATUS_REG` register provides information about the availability of new data for the accelerometer, gyroscope, and temperature sensor.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::StatusRegOrSpiaux, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusReg {
    #[bits(1, access = RO)]
    pub xlda: u8,
    #[bits(1, access = RO)]
    pub gda: u8,
    #[bits(1, access = RO)]
    pub tda: u8,
    #[bits(5, access = RO)]
    not_used_01: u8,
}

/// Status register for auxiliary SPI (R).
///
/// The `STATUS_SPIAUX` register provides information about the availability of new data for the accelerometer and gyroscope, as well as the gyroscope settling status.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::StatusRegOrSpiaux, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusSpiaux {
    #[bits(1, access = RO)]
    pub xlda: u8,
    #[bits(1, access = RO)]
    pub gda: u8,
    #[bits(1, access = RO)]
    pub gyro_settling: u8,
    #[bits(5, access = RO)]
    not_used_01: u8,
}

/// Embedded function status register (R).
///
/// The `EMB_FUNC_STATUS_MAINPAGE` register provides the status of various embedded function events, such as step detection, tilt detection, and significant motion detection.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::EmbFuncStatusMainpage, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncStatusMainpage {
    #[bits(3, access = RO)]
    not_used_01: u8,
    #[bits(1, access = RO)]
    pub is_step_det: u8,
    #[bits(1, access = RO)]
    pub is_tilt: u8,
    #[bits(1, access = RO)]
    pub is_sigmot: u8,
    #[bits(1, access = RO)]
    not_used_02: u8,
    #[bits(1, access = RO)]
    pub is_fsm_lc: u8,
}

/// Finite state machine status register A (R).
///
/// The `FSM_STATUS_A_MAINPAGE` register provides the status of FSM programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FsmStatusAMainpage, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmStatusAMainpage {
    #[bits(1, access = RO)]
    pub is_fsm1: u8,
    #[bits(1, access = RO)]
    pub is_fsm2: u8,
    #[bits(1, access = RO)]
    pub is_fsm3: u8,
    #[bits(1, access = RO)]
    pub is_fsm4: u8,
    #[bits(1, access = RO)]
    pub is_fsm5: u8,
    #[bits(1, access = RO)]
    pub is_fsm6: u8,
    #[bits(1, access = RO)]
    pub is_fsm7: u8,
    #[bits(1, access = RO)]
    pub is_fsm8: u8,
}

/// Finite state machine status register B (R).
///
/// The `FSM_STATUS_B_MAINPAGE` register provides the status of FSM programs 9 through 16.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FsmStatusBMainpage, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmStatusBMainpage {
    #[bits(1, access = RO)]
    pub is_fsm9: u8,
    #[bits(1, access = RO)]
    pub is_fsm10: u8,
    #[bits(1, access = RO)]
    pub is_fsm11: u8,
    #[bits(1, access = RO)]
    pub is_fsm12: u8,
    #[bits(1, access = RO)]
    pub is_fsm13: u8,
    #[bits(1, access = RO)]
    pub is_fsm14: u8,
    #[bits(1, access = RO)]
    pub is_fsm15: u8,
    #[bits(1, access = RO)]
    pub is_fsm16: u8,
}

/// Machine learning core status register (R).
///
/// The `MLC_STATUS_MAINPAGE` register provides the status of MLC programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::MlcStatusMainpage, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcStatusMainpage {
    #[bits(1, access = RO)]
    pub is_mlc1: u8,
    #[bits(1, access = RO)]
    pub is_mlc2: u8,
    #[bits(1, access = RO)]
    pub is_mlc3: u8,
    #[bits(1, access = RO)]
    pub is_mlc4: u8,
    #[bits(1, access = RO)]
    pub is_mlc5: u8,
    #[bits(1, access = RO)]
    pub is_mlc6: u8,
    #[bits(1, access = RO)]
    pub is_mlc7: u8,
    #[bits(1, access = RO)]
    pub is_mlc8: u8,
}

/// Sensor hub status register (R).
///
/// The `STATUS_MASTER_MAINPAGE` register provides the status of the sensor hub, including the completion of operations and NACK errors for slave devices.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::StatusMasterMainpage, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusMasterMainpage {
    #[bits(1, access = RO)]
    pub sens_hub_endop: u8,
    #[bits(2, access = RO)]
    not_used_01: u8,
    #[bits(1, access = RO)]
    pub slave0_nack: u8,
    #[bits(1, access = RO)]
    pub slave1_nack: u8,
    #[bits(1, access = RO)]
    pub slave2_nack: u8,
    #[bits(1, access = RO)]
    pub slave3_nack: u8,
    #[bits(1, access = RO)]
    pub wr_once_done: u8,
}

/// FIFO status register 1 (R).
///
/// The `FIFO_STATUS1` register provides the lower 8 bits of the FIFO data level.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FifoStatus1, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoStatus1 {
    #[bits(8, access = RO)]
    pub diff_fifo: u8,
}

/// FIFO status register 2 (R).
///
/// The `FIFO_STATUS2` register provides the upper 2 bits of the FIFO data level and various FIFO status flags.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FifoStatus2, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoStatus2 {
    #[bits(2, access = RO)]
    pub diff_fifo: u8,
    #[bits(1, access = RO)]
    not_used_01: u8,
    #[bits(1, access = RO)]
    pub over_run_latched: u8,
    #[bits(1, access = RO)]
    pub counter_bdr_ia: u8,
    #[bits(1, access = RO)]
    pub fifo_full_ia: u8,
    #[bits(1, access = RO)]
    pub fifo_ovr_ia: u8,
    #[bits(1, access = RO)]
    pub fifo_wtm_ia: u8,
}

#[register(address = Reg::Timestamp0, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u32, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u32, order = Lsb))]
pub struct TimestampReg {
    #[bits(32, access = RO)]
    pub timestamp: u32,
}

/// Tap configuration register 0 (R/W).
///
/// The `TAP_CFG0` register is used to configure tap detection settings, such as enabling tap detection on specific axes and configuring interrupt behavior.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::TapCfg0, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapCfg0 {
    #[bits(1, default = 0)]
    pub lir: u8,
    #[bits(1, default = 0)]
    pub tap_z_en: u8,
    #[bits(1, default = 0)]
    pub tap_y_en: u8,
    #[bits(1, default = 0)]
    pub tap_x_en: u8,
    #[bits(1, default = 0)]
    pub slope_fds: u8,
    #[bits(1, default = 0)]
    pub sleep_status_on_int: u8,
    #[bits(1, default = 0)]
    pub int_clr_on_read: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
}

/// Tap configuration register 1 (R/W).
///
/// The `TAP_CFG1` register is used to configure the X-axis tap threshold and axis priority for tap detection.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::TapCfg1, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapCfg1 {
    #[bits(5, default = 0)]
    pub tap_ths_x: u8,
    #[bits(3, default = 0)]
    pub tap_priority: u8,
}

/// Tap configuration register 2 (R/W).
///
/// The `TAP_CFG2` register is used to configure the Y-axis tap threshold, inactivity mode, and interrupt enable settings.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::TapCfg2, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapCfg2 {
    #[bits(5, default = 0)]
    pub tap_ths_y: u8,
    #[bits(2, default = 0)]
    pub inact_en: u8,
    #[bits(1, default = 0)]
    pub interrupts_enable: u8,
}

/// Tap threshold and 6D configuration register (R/W).
///
/// The `TAP_THS_6D` register is used to configure the Z-axis tap threshold, 6D orientation threshold, and 4D orientation enable settings.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::TapThs6d, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TapThs6d {
    #[bits(5, default = 0)]
    pub tap_ths_z: u8,
    #[bits(2, default = 0)]
    pub sixd_ths: u8,
    #[bits(1, default = 0)]
    pub d4d_en: u8,
}

/// Interrupt duration register 2 (R/W).
///
/// The `INT_DUR2` register is used to configure the shock, quiet, and duration settings for tap detection.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::IntDur2, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntDur2 {
    #[bits(2, default = 0)]
    pub shock: u8,
    #[bits(2, default = 0)]
    pub quiet: u8,
    #[bits(4, default = 0)]
    pub dur: u8,
}

/// Wake-up threshold register (R/W).
///
/// The `WAKE_UP_THS` register is used to configure the wake-up threshold, user offset weight, and single/double-tap enable settings.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::WakeUpThs, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WakeUpThs {
    #[bits(6, default = 0)]
    pub wk_ths: u8,
    #[bits(1, default = 0)]
    pub usr_off_on_wu: u8,
    #[bits(1, default = 0)]
    pub single_double_tap: u8,
}

/// Wake-up duration register (R/W).
///
/// The `WAKE_UP_DUR` register is used to configure the sleep duration, wake-up duration, and free-fall duration settings.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::WakeUpDur, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WakeUpDur {
    #[bits(4, default = 0)]
    pub sleep_dur: u8,
    #[bits(1, default = 0)]
    pub wake_ths_w: u8,
    #[bits(2, default = 0)]
    pub wake_dur: u8,
    #[bits(1, default = 0)]
    pub ff_dur: u8,
}

/// Free-fall configuration register (R/W).
///
/// The `FREE_FALL` register is used to configure the free-fall threshold and duration settings.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FreeFall, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FreeFall {
    #[bits(3, default = 0)]
    pub ff_ths: u8,
    #[bits(5, default = 0)]
    pub ff_dur: u8,
}

/// MD1 configuration register (R/W).
///
/// The `MD1_CFG` register is used to configure interrupt routing to the INT1 pin for various events, such as wake-up, free-fall, and tap detection.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Md1Cfg, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Md1Cfg {
    #[bits(1, default = 0)]
    pub int1_shub: u8,
    #[bits(1, default = 0)]
    pub int1_emb_func: u8,
    #[bits(1, default = 0)]
    pub int1_6d: u8,
    #[bits(1, default = 0)]
    pub int1_double_tap: u8,
    #[bits(1, default = 0)]
    pub int1_ff: u8,
    #[bits(1, default = 0)]
    pub int1_wu: u8,
    #[bits(1, default = 0)]
    pub int1_single_tap: u8,
    #[bits(1, default = 0)]
    pub int1_sleep_change: u8,
}

/// MD2 configuration register (R/W).
///
/// The `MD2_CFG` register is used to configure interrupt routing to the INT2 pin for various events, such as wake-up, free-fall, and tap detection.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Md2Cfg, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Md2Cfg {
    #[bits(1, default = 0)]
    pub int2_timestamp: u8,
    #[bits(1, default = 0)]
    pub int2_emb_func: u8,
    #[bits(1, default = 0)]
    pub int2_6d: u8,
    #[bits(1, default = 0)]
    pub int2_double_tap: u8,
    #[bits(1, default = 0)]
    pub int2_ff: u8,
    #[bits(1, default = 0)]
    pub int2_wu: u8,
    #[bits(1, default = 0)]
    pub int2_single_tap: u8,
    #[bits(1, default = 0)]
    pub int2_sleep_change: u8,
}

/// Internal frequency fine adjustment register (R).
///
/// The `INTERNAL_FREQ_FINE` register provides the fine adjustment value for the internal frequency.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::InternalFreqFine, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct InternalFreqFine {
    #[bits(8, access = RO)]
    pub freq_fine: i8,
}

/// Interrupt OIS register (R).
/// Only Aux SPI can write to this register (R/W).
///
/// The `INT_OIS` register provides information about OIS (Optical Image Stabilization) interrupt settings and status.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::IntOis, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntOis {
    #[bits(2, default = 0)]
    pub st_xl_ois: u8,
    #[bits(3, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub den_lh_ois: u8,
    #[bits(1, default = 0)]
    pub lvl2_ois: u8,
    #[bits(1, default = 0)]
    pub int2_drdy_ois: u8,
}

/// Control register 1 for OIS (R).
/// Only Aux SPI can write to this register (R/W).
///
/// The `CTRL1_OIS` register provides configuration for OIS settings, such as enabling OIS on SPI2 and setting full-scale ranges.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl1Ois, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl1Ois {
    #[bits(1, default = 0)]
    pub ois_en_spi2: u8,
    #[bits(1, default = 0)]
    pub fs_125_ois: u8,
    #[bits(2, default = 0)]
    pub fs_g_ois: u8,
    #[bits(1, default = 0)]
    pub mode4_en: u8,
    #[bits(1, default = 0)]
    pub sim_ois: u8,
    #[bits(1, default = 0)]
    pub lvl1_ois: u8,
    #[bits(1, default = 0)]
    not_used_01: u8,
}

/// Control register 2 for OIS (R).
/// Only Aux SPI can write to this register (R/W).
///
/// The `CTRL2_OIS` register provides configuration for OIS high-pass filters and filter types.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl2Ois, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl2Ois {
    #[bits(1, default = 0)]
    pub hp_en_ois: u8,
    #[bits(2, default = 0)]
    pub ftype_ois: u8,
    #[bits(1, default = 0)]
    not_used_01: u8,
    #[bits(2, default = 0)]
    pub hpm_ois: u8,
    #[bits(2, default = 0)]
    not_used_02: u8,
}

/// Control register 3 for OIS (R).
/// Only Aux SPI can write to this register (R/W).
///
/// The `CTRL3_OIS` register provides configuration for OIS self-test and accelerometer filter settings.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::Ctrl3Ois, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl3Ois {
    #[bits(1, default = 0)]
    pub st_ois_clampdis: u8,
    #[bits(2, default = 0)]
    pub st_ois: u8,
    #[bits(3, default = 0)]
    pub filter_xl_conf_ois: u8,
    #[bits(2, default = 0)]
    pub fs_xl_ois: u8,
}

/// Accelerometer X-axis user offser correction (X_OFS_USR) (R/W).
///
/// The offset value set in X_OFS_USR is internally subtracted from the acceleration value measured
/// on the X-axis.
#[register(address = Reg::XOfsUsr, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct XOfsUsr {
    #[bits(8, default = 0)]
    pub x_ofs_usr: i8,
}

/// Accelerometer Y-axis user offser correction (Y_OFS_USR) (R/W).
///
/// The offset value set in Y_OFS_USR is internally subtracted from the acceleration value measured
/// on the Y-axis.
#[register(address = Reg::YOfsUsr, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct YOfsUsr {
    #[bits(8, default = 0)]
    pub y_ofs_usr: i8,
}

/// Accelerometer Z-axis user offser correction (Z_OFS_USR) (R/W).
///
/// The offset value set in Z_OFS_USR is internally subtracted from the acceleration value measured
/// on the Z-axis.
#[register(address = Reg::ZOfsUsr, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct ZOfsUsr {
    #[bits(8, default = 0)]
    pub z_ofs_usr: i8,
}

/// FIFO data output tag register (R).
///
/// The `FIFO_DATA_OUT_TAG` register provides information about the FIFO tag, including parity, count, and sensor type.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = Reg::FifoDataOutTag, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoDataOutTag {
    #[bits(1, access = RO)]
    pub tag_parity: u8,
    #[bits(2, access = RO)]
    pub tag_cnt: u8,
    #[bits(5, access = RO)]
    pub tag_sensor: u8,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsXl {
    #[default]
    _2g = 0,
    _16g = 1,
    _4g = 2,
    _8g = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, Debug, TryFrom)]
#[try_from(repr)]
pub enum OdrXl {
    #[default]
    Off = 0,
    _12_5hz = 1,
    _26hz = 2,
    _52hz = 3,
    _104hz = 4,
    _208hz = 5,
    _416hz = 6,
    _833hz = 7,
    _1666hz = 8,
    _3332hz = 9,
    _6667hz = 10,
    _1_6hz = 11,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsGy {
    _125dps = 2,
    #[default]
    _250dps = 0,
    _500dps = 4,
    _1000dps = 8,
    _2000dps = 12,
    _4000dps = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, Debug, TryFrom)]
#[try_from(repr)]
pub enum OdrGy {
    #[default]
    Off = 0,
    _12_5hz = 1,
    _26hz = 2,
    _52hz = 3,
    _104hz = 4,
    _208hz = 5,
    _416hz = 6,
    _833hz = 7,
    _1666hz = 8,
    _3332hz = 9,
    _6667hz = 10,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum UsrOffW {
    #[default]
    _1mg = 0,
    _16mg = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum XlHmMode {
    #[default]
    HighPerformance = 0,
    LowNormalPower = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum GyHmMode {
    #[default]
    HighPerformance = 0,
    Normal = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Rounding {
    #[default]
    NoRound = 0,
    RoundXl = 1,
    RoundGy = 2,
    RoundGyXl = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum DatareadyPulsed {
    #[default]
    Latched = 0,
    Pulsed = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum StXl {
    #[default]
    Disable = 0,
    Positive = 1,
    Negative = 2,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum StGy {
    #[default]
    Disable = 0,
    Positive = 1,
    Negative = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Ftype {
    #[default]
    UltraLight = 0,
    VeryLight = 1,
    Light = 2,
    Medium = 3,
    Strong = 4,
    VeryStrong = 5,
    Aggressive = 6,
    Xtreme = 7,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum HpSlopeXlEn {
    #[default]
    HpPathDisableOnOut = 0x00,
    SlopeOdrDiv4 = 0x10,
    HpOdrDiv10 = 0x11,
    HpOdrDiv20 = 0x12,
    HpOdrDiv45 = 0x13,
    HpOdrDiv100 = 0x14,
    HpOdrDiv200 = 0x15,
    HpOdrDiv400 = 0x16,
    HpOdrDiv800 = 0x17,
    HpRefMdOdrDiv10 = 0x31,
    HpRefMdOdrDiv20 = 0x32,
    HpRefMdOdrDiv45 = 0x33,
    HpRefMdOdrDiv100 = 0x34,
    HpRefMdOdrDiv200 = 0x35,
    HpRefMdOdrDiv400 = 0x36,
    HpRefMdOdrDiv800 = 0x37,
    LpOdrDiv10 = 0x01,
    LpOdrDiv20 = 0x02,
    LpOdrDiv45 = 0x03,
    LpOdrDiv100 = 0x04,
    LpOdrDiv200 = 0x05,
    LpOdrDiv400 = 0x06,
    LpOdrDiv800 = 0x07,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SlopeFds {
    #[default]
    UseSlope = 0,
    UseHpf = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum HpmGy {
    #[default]
    None = 0x00,
    _16mhz = 0x80,
    _65mhz = 0x81,
    _260mhz = 0x82,
    _1_04hz = 0x83,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OisPuDis {
    #[default]
    Disc = 1,
    Connect = 0,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OisOn {
    #[default]
    Off = 0,
    PrimaryInterfaceOn = 3,
    AuxInterfaceOn = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum StXlOis {
    #[default]
    Disable = 0,
    Pos = 1,
    Neg = 2,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum DenLhOis {
    #[default]
    Low = 0,
    High = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Lvl2Ois {
    #[default]
    Disable = 0,
    LevelLatch = 3,
    LevelTrig = 2,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OisEnSpi2 {
    #[default]
    AuxDisable = 0,
    Mode3Gy = 1,
    Mode4GyXl = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsGOis {
    _125dpsAux = 0x04,
    #[default]
    _250dpsAux = 0x00,
    _500dpsAux = 0x01,
    _1000dpsAux = 0x02,
    _2000dpsAux = 0x03,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SimOis {
    #[default]
    Spi4Wire = 0,
    Spi3Wire = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FtypeOis {
    #[default]
    _297hz = 0,
    _222hz = 1,
    _154hz = 2,
    _470hz = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum HpmOis {
    #[default]
    Disable = 0x00,
    _16mhz = 0x10,
    _65mhz = 0x11,
    _260mhz = 0x12,
    _1_04hz = 0x13,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum StOisClampdis {
    #[default]
    EnableClamp = 0,
    DisableClamp = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum StOis {
    #[default]
    Disable = 0,
    Pos = 1,
    Neg = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FilterXlConfOis {
    #[default]
    _631hz = 0,
    _295hz = 1,
    _140hz = 2,
    _68_2hz = 3,
    _33_6hz = 4,
    _16_7hz = 5,
    _8_3hz = 6,
    _4_14hz = 7,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsXlOis {
    #[default]
    _2g = 0,
    _16g = 1,
    _4g = 2,
    _8g = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SdoPuEn {
    #[default]
    Disc = 0,
    Connect = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Sim {
    #[default]
    Spi4Wire = 0,
    Spi3Wire = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum I2cDisable {
    #[default]
    Enable = 0,
    Disable = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum PpOd {
    #[default]
    PushPull = 0,
    OpenDrain = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum HLactive {
    #[default]
    High = 0,
    Low = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Lir {
    #[default]
    AllIntPulsed = 0,
    BaseLatchedEmbPulsed = 1,
    BasePulsedEmbLatched = 2,
    AllIntLatched = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum WakeThsW {
    #[default]
    FsDiv64 = 0,
    FsDiv256 = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SleepStatusOnInt {
    #[default]
    DriveSleepChgEvent = 0,
    DriveSleepStatus = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum InactEn {
    #[default]
    XlAndGyNotAffected = 0,
    Xl12hz5GyNotAffected = 1,
    Xl12hz5GySleep = 2,
    Xl12hz5GyPd = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, Debug)]
pub enum TapPriority {
    #[default]
    Xyz = 0,
    Yxz = 1,
    Xzy = 2,
    Zyx = 3,
    Yzx = 5,
    Zxy = 6,
}

impl TryFrom<u8> for TapPriority {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b000 => Ok(TapPriority::Xyz),
            0b001 => Ok(TapPriority::Yxz),
            0b010 => Ok(TapPriority::Xzy),
            0b011 => Ok(TapPriority::Zyx),
            0b100 => Ok(TapPriority::Xyz),
            0b101 => Ok(TapPriority::Yzx),
            0b110 => Ok(TapPriority::Zxy),
            0b111 => Ok(TapPriority::Zyx),
            _ => Err(()),
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SingleDoubleTap {
    #[default]
    OnlySingle = 0,
    BothSingleDouble = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SixdThs {
    #[default]
    _80deg = 0,
    _70deg = 1,
    _60deg = 2,
    _50deg = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FfThs {
    #[default]
    _156mg = 0,
    _219mg = 1,
    _250mg = 2,
    _312mg = 3,
    _344mg = 4,
    _406mg = 5,
    _469mg = 6,
    _500mg = 7,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum BdrXl {
    #[default]
    NotBatched = 0,
    _12_5hz = 1,
    _26hz = 2,
    _52hz = 3,
    _104hz = 4,
    _208hz = 5,
    _417hz = 6,
    _833hz = 7,
    _1667hz = 8,
    _3333hz = 9,
    _6667hz = 10,
    _1_6hz = 11,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum BdrGy {
    #[default]
    NotBatched = 0,
    _12_5hz = 1,
    _26hz = 2,
    _52hz = 3,
    _104hz = 4,
    _208hz = 5,
    _417hz = 6,
    _833hz = 7,
    _1667hz = 8,
    _3333hz = 9,
    _6667hz = 10,
    _6_5hz = 11,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoMode {
    #[default]
    BypassMode = 0,
    FifoMode = 1,
    StreamToFifoMode = 3,
    BypassToStreamMode = 4,
    StreamMode = 6,
    BypassToFifoMode = 7,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OdrTBatch {
    #[default]
    NotBatched = 0,
    _1_6hz = 1,
    _12_5hz = 2,
    _52hz = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum OdrTsBatch {
    #[default]
    NoDecimation = 0,
    _1 = 1,
    _8 = 2,
    _32 = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum TrigCounterBdr {
    #[default]
    XlBatchEvent = 0,
    GyroBatchEvent = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoTag {
    GyroNcTag = 0x01,
    XlNcTag = 0x02,
    TemperatureTag = 0x03,
    TimestampTag = 0x04,
    CfgChangeTag = 0x05,
    XlNcT2 = 0x06,
    XlNcT1 = 0x07,
    Xl2xC = 0x08,
    Xl3xC = 0x09,
    GyroNcT2 = 0x0A,
    GyroNcT1 = 0x0B,
    Gyro2xC = 0x0C,
    Gyro3xC = 0x0D,
    SensorHubSlave0 = 0x0E,
    SensorHubSlave1 = 0x0F,
    SensorHubSlave2 = 0x10,
    SensorHubSlave3 = 0x11,
    StepCounter = 0x12,
    #[default]
    SensorhubNackTag = 0x19,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum DenMode {
    #[default]
    Disable = 0,
    LevelFifo = 6,
    LevelLetched = 3,
    LevelTrigger = 2,
    EdgeTrigger = 4,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum DenLh {
    #[default]
    Low = 0,
    High = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum DenXlG {
    #[default]
    GyData = 0,
    XlData = 2,
    GyXlData = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum CarryCountEn {
    #[default]
    EveryStep = 0,
    CountOverflow = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum MagZAxis {
    ZEqY = 0,
    ZEqMinY = 1,
    ZEqX = 2,
    ZEqMinX = 3,
    ZEqMinZ = 4,
    #[default]
    ZEqZ = 5,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum MagYAxis {
    #[default]
    YEqY = 0,
    YEqMinY = 1,
    YEqX = 2,
    YEqMinX = 3,
    YEqMinZ = 4,
    YEqZ = 5,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum MagXAxis {
    XEqY = 0,
    XEqMinY = 1,
    #[default]
    XEqX = 2,
    XEqMinX = 3,
    XEqMinZ = 4,
    XEqZ = 5,
}
