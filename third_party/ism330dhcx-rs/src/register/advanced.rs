use super::super::{
    BusOperation, DelayNs, EmbAdvFunctions, Error, Ism330dhcx, RegisterOperation, bisync,
    register::MainBank,
};

use bitfield_struct::bitfield;
use st_mem_bank_macro::adv_register;

#[repr(u16)]
pub enum AdvPage {
    _0 = 0x0,
    _1 = 0x100,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum EmbAdvReg0 {
    /// Magnetometer sensitivity register (low byte).
    ///
    /// Configures the low byte of the magnetometer sensitivity.
    MagSensitivityL = 0xBA,

    /// Magnetometer sensitivity register (high byte).
    ///
    /// Configures the high byte of the magnetometer sensitivity.
    MagSensitivityH = 0xBB,

    /// Magnetometer X-axis offset register (low byte).
    ///
    /// Configures the low byte of the X-axis offset for hard-iron compensation.
    MagOffxL = 0xC0,

    /// Magnetometer X-axis offset register (high byte).
    ///
    /// Configures the high byte of the X-axis offset for hard-iron compensation.
    MagOffxH = 0xC1,

    /// Magnetometer Y-axis offset register (low byte).
    ///
    /// Configures the low byte of the Y-axis offset for hard-iron compensation.
    MagOffyL = 0xC2,

    /// Magnetometer Y-axis offset register (high byte).
    ///
    /// Configures the high byte of the Y-axis offset for hard-iron compensation.
    MagOffyH = 0xC3,

    /// Magnetometer Z-axis offset register (low byte).
    ///
    /// Configures the low byte of the Z-axis offset for hard-iron compensation.
    MagOffzL = 0xC4,

    /// Magnetometer Z-axis offset register (high byte).
    ///
    /// Configures the high byte of the Z-axis offset for hard-iron compensation.
    MagOffzH = 0xC5,

    /// Magnetometer soft-iron correction matrix element XX (low byte).
    MagSiXxL = 0xC6,

    /// Magnetometer soft-iron correction matrix element XX (high byte).
    MagSiXxH = 0xC7,

    /// Magnetometer soft-iron correction matrix element XY (low byte).
    MagSiXyL = 0xC8,

    /// Magnetometer soft-iron correction matrix element XY (high byte).
    MagSiXyH = 0xC9,

    /// Magnetometer soft-iron correction matrix element XZ (low byte).
    MagSiXzL = 0xCA,

    /// Magnetometer soft-iron correction matrix element XZ (high byte).
    MagSiXzH = 0xCB,

    /// Magnetometer soft-iron correction matrix element YY (low byte).
    MagSiYyL = 0xCC,

    /// Magnetometer soft-iron correction matrix element YY (high byte).
    MagSiYyH = 0xCD,

    /// Magnetometer soft-iron correction matrix element YZ (low byte).
    MagSiYzL = 0xCE,

    /// Magnetometer soft-iron correction matrix element YZ (high byte).
    MagSiYzH = 0xCF,

    /// Magnetometer soft-iron correction matrix element ZZ (low byte).
    MagSiZzL = 0xD0,

    /// Magnetometer soft-iron correction matrix element ZZ (high byte).
    MagSiZzH = 0xD1,

    /// Magnetometer configuration register A.
    ///
    /// Configures specific magnetometer settings.
    MagCfgA = 0xD4,

    /// Magnetometer configuration register B.
    ///
    /// Configures additional magnetometer settings.
    MagCfgB = 0xD5,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum EmbAdvReg1 {
    /// FSM long counter timeout register (low byte).
    ///
    /// Configures the low byte of the FSM long counter timeout.
    FsmLcTimeoutL = 0x7A,

    /// FSM long counter timeout register (high byte).
    ///
    /// Configures the high byte of the FSM long counter timeout.
    FsmLcTimeoutH = 0x7B,

    /// FSM programs register.
    ///
    /// Configures the FSM programs to be executed.
    FsmPrograms = 0x7C,

    /// FSM start address register (low byte).
    ///
    /// Configures the low byte of the FSM start address.
    FsmStartAddL = 0x7E,

    /// FSM start address register (high byte).
    ///
    /// Configures the high byte of the FSM start address.
    FsmStartAddH = 0x7F,

    /// Pedometer command register.
    ///
    /// Configures pedometer-specific commands.
    PedoCmdReg = 0x83,

    /// Pedometer debounce steps configuration register.
    ///
    /// Configures the debounce threshold for step detection.
    PedoDebStepsConf = 0x84,

    /// Pedometer step count delta time register (low byte).
    ///
    /// Configures the low byte of the step count delta time.
    PedoScDeltatL = 0xD0,

    /// Pedometer step count delta time register (high byte).
    ///
    /// Configures the high byte of the step count delta time.
    PedoScDeltatH = 0xD1,

    /// MLC magnetometer sensitivity register (low byte).
    ///
    /// Configures the low byte of the magnetometer sensitivity for MLC.
    MlcMagSensitivityL = 0xE8,

    /// MLC magnetometer sensitivity register (high byte).
    ///
    /// Configures the high byte of the magnetometer sensitivity for MLC.
    MlcMagSensitivityH = 0xE9,
}

/// Magnetometer configuration register A (R/W).
///
/// The `MAG_CFG_A` register is used to configure the Z and Y axes of the magnetometer.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[adv_register(base_address = AdvPage::_0, address = EmbAdvReg0::MagCfgA, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MagCfgA {
    #[bits(3, default = 0b101)]
    pub mag_z_axis: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(3, default = 0)]
    pub mag_y_axis: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_02: u8,
}

/// Magnetometer configuration register B (R/W).
///
/// The `MAG_CFG_B` register is used to configure the X axis of the magnetometer.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[adv_register(base_address = AdvPage::_0, address = EmbAdvReg0::MagCfgB, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MagCfgB {
    #[bits(3, default = 0b010)]
    pub mag_x_axis: u8,
    #[bits(5, access = RO, default = 0)]
    not_used_01: u8,
}

#[adv_register(base_address = AdvPage::_0, address = EmbAdvReg0::MagSensitivityL, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct MagSensitivity {
    #[bits(16, default = 0b0001011000100100)]
    pub mag_s: u16,
}

#[adv_register(base_address = AdvPage::_0, address = EmbAdvReg0::MagOffxL, access_type = "Ism330dhcx<B, T, MainBank>")]
pub struct MagOffXYZ(pub [i16; 3]);

/// XX XY XZ YY YZ ZZ
#[adv_register(base_address = AdvPage::_0, address = EmbAdvReg0::MagSiXxL, access_type = "Ism330dhcx<B, T, MainBank>")]
pub struct MagSiMatrix(pub [u16; 6]);

#[adv_register(base_address = AdvPage::_1, address = EmbAdvReg1::FsmLcTimeoutL, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct FsmLcTimeout {
    #[bits(16, default = 0)]
    pub fsm_lc_timeout: u16,
}

#[adv_register(base_address = AdvPage::_1, address = EmbAdvReg1::FsmPrograms, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmPrograms {
    #[bits(8, default = 0)]
    pub fsm_prog: u8,
}

#[adv_register(base_address = AdvPage::_1, address = EmbAdvReg1::FsmStartAddL, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct FsmStartAdd {
    #[bits(16, default = 0)]
    pub fsm_start: u16,
}

/// Pedometer command register (R/W).
///
/// The `PEDO_CMD_REG` register is used to enable carry count functionality for the pedometer.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[adv_register(base_address = AdvPage::_1, address = EmbAdvReg1::PedoCmdReg, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PedoCmdReg {
    #[bits(3, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub carry_count_en: u8,
    #[bits(4, access = RO, default = 0)]
    not_used_02: u8,
}

#[adv_register(base_address = AdvPage::_1, address = EmbAdvReg1::PedoDebStepsConf, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PedoDebStepsConf {
    #[bits(8, default = 0b00001010)]
    pub deb_step: u8,
}

#[adv_register(base_address = AdvPage::_1, address = EmbAdvReg1::PedoScDeltatL, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct PedoScDeltat {
    #[bits(16)]
    pub pd_sc: u16,
}

#[adv_register(base_address = AdvPage::_1, address = EmbAdvReg1::MlcMagSensitivityL, access_type = "Ism330dhcx<B, T, MainBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct MlcMagSensitivity {
    #[bits(16, default = 0b0011110000000000)]
    pub mlc_mag_s: u16,
}
