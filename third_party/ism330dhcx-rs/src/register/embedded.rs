use super::super::{
    BusOperation, DelayNs, Error, Ism330dhcx, RegisterOperation, SensorOperation, bisync,
    register::EmbBank,
};

use bitfield_struct::bitfield;
use derive_more::TryFrom;
use st_mem_bank_macro::register;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum EmbReg {
    /// Page selection register.
    ///
    /// Used to select the advanced feature page for accessing additional registers.
    PageSel = 0x02,

    /// Embedded function enable register A.
    ///
    /// Enables specific embedded functions, such as pedometer and tilt detection.
    EmbFuncEnA = 0x04,

    /// Embedded function enable register B.
    ///
    /// Enables additional embedded functions, such as FSM and MLC.
    EmbFuncEnB = 0x05,

    /// Page address register.
    ///
    /// Specifies the address of the register to be accessed on the selected page.
    PageAddress = 0x08,

    /// Page value register.
    ///
    /// Holds the value to be written to or read from the selected page register.
    PageValue = 0x09,

    /// Embedded function interrupt 1 configuration register.
    ///
    /// Configures events routed to the INT1 pin for embedded functions.
    EmbFuncInt1 = 0x0A,

    /// FSM interrupt 1 configuration register A.
    ///
    /// Configures FSM programs 1–8 to generate interrupts on INT1.
    FsmInt1A = 0x0B,

    /// FSM interrupt 1 configuration register B.
    ///
    /// Configures FSM programs 9–16 to generate interrupts on INT1.
    FsmInt1B = 0x0C,

    /// MLC interrupt 1 configuration register.
    ///
    /// Configures MLC events to generate interrupts on INT1.
    MlcInt1 = 0x0D,

    /// Embedded function interrupt 2 configuration register.
    ///
    /// Configures events routed to the INT2 pin for embedded functions.
    EmbFuncInt2 = 0x0E,

    /// FSM interrupt 2 configuration register A.
    ///
    /// Configures FSM programs 1–8 to generate interrupts on INT2.
    FsmInt2A = 0x0F,

    /// FSM interrupt 2 configuration register B.
    ///
    /// Configures FSM programs 9–16 to generate interrupts on INT2.
    FsmInt2B = 0x10,

    /// MLC interrupt 2 configuration register.
    ///
    /// Configures MLC events to generate interrupts on INT2.
    MlcInt2 = 0x11,

    /// Embedded function status register.
    ///
    /// Provides the status of embedded functions, such as pedometer and tilt detection.
    EmbFuncStatus = 0x12,

    /// FSM status register A.
    ///
    /// Provides the status of FSM programs 1–8.
    FsmStatusA = 0x13,

    /// FSM status register B.
    ///
    /// Provides the status of FSM programs 9–16.
    FsmStatusB = 0x14,

    /// MLC status register.
    ///
    /// Provides the status of MLC events.
    MlcStatus = 0x15,

    /// Page read/write register.
    ///
    /// Configures read/write operations for the selected page.
    PageRw = 0x17,

    /// Embedded function FIFO configuration register.
    ///
    /// Configures the FIFO for embedded function data storage.
    EmbFuncFifoCfg = 0x44,

    /// FSM enable register A.
    ///
    /// Enables FSM programs 1–8.
    FsmEnableA = 0x46,

    /// FSM enable register B.
    ///
    /// Enables FSM programs 9–16.
    FsmEnableB = 0x47,

    /// FSM long counter low byte register.
    ///
    /// Configures the low byte of the FSM long counter timeout.
    FsmLongCounterL = 0x48,

    /// FSM long counter high byte register.
    ///
    /// Configures the high byte of the FSM long counter timeout.
    FsmLongCounterH = 0x49,

    /// FSM long counter clear register.
    ///
    /// Clears the FSM long counter.
    FsmLongCounterClear = 0x4A,

    /// FSM output register 1.
    ///
    /// Provides the output of FSM program 1.
    FsmOuts1 = 0x4C,

    /// FSM output register 2.
    ///
    /// Provides the output of FSM program 2.
    FsmOuts2 = 0x4D,

    /// FSM output register 3.
    ///
    /// Provides the output of FSM program 3.
    FsmOuts3 = 0x4E,

    /// FSM output register 4.
    ///
    /// Provides the output of FSM program 4.
    FsmOuts4 = 0x4F,

    /// FSM output register 5.
    ///
    /// Provides the output of FSM program 5.
    FsmOuts5 = 0x50,

    /// FSM output register 6.
    ///
    /// Provides the output of FSM program 6.
    FsmOuts6 = 0x51,

    /// FSM output register 7.
    ///
    /// Provides the output of FSM program 7.
    FsmOuts7 = 0x52,

    /// FSM output register 8.
    ///
    /// Provides the output of FSM program 8.
    FsmOuts8 = 0x53,

    /// FSM output register 9.
    ///
    /// Provides the output of FSM program 9.
    FsmOuts9 = 0x54,

    /// FSM output register 10.
    ///
    /// Provides the output of FSM program 10.
    FsmOuts10 = 0x55,

    /// FSM output register 11.
    ///
    /// Provides the output of FSM program 11.
    FsmOuts11 = 0x56,

    /// FSM output register 12.
    ///
    /// Provides the output of FSM program 12.
    FsmOuts12 = 0x57,

    /// FSM output register 13.
    ///
    /// Provides the output of FSM program 13.
    FsmOuts13 = 0x58,

    /// FSM output register 14.
    ///
    /// Provides the output of FSM program 14.
    FsmOuts14 = 0x59,

    /// FSM output register 15.
    ///
    /// Provides the output of FSM program 15.
    FsmOuts15 = 0x5A,

    /// FSM output register 16.
    ///
    /// Provides the output of FSM program 16.
    FsmOuts16 = 0x5B,

    /// Embedded function output data rate configuration register B.
    ///
    /// Configures the output data rate for embedded functions.
    EmbFuncOdrCfgB = 0x5F,

    /// Embedded function output data rate configuration register C.
    ///
    /// Configures additional output data rate settings for embedded functions.
    EmbFuncOdrCfgC = 0x60,

    /// Step counter low byte register.
    ///
    /// Provides the low byte of the step counter value.
    StepCounterL = 0x62,

    /// Step counter high byte register.
    ///
    /// Provides the high byte of the step counter value.
    StepCounterH = 0x63,

    /// Embedded function source register.
    ///
    /// Provides the source of embedded function events.
    EmbFuncSrc = 0x64,

    /// Embedded function initialization register A.
    ///
    /// Configures initialization settings for embedded functions.
    EmbFuncInitA = 0x66,

    /// Embedded function initialization register B.
    ///
    /// Configures additional initialization settings for embedded functions.
    EmbFuncInitB = 0x67,

    /// MLC source register 0.
    ///
    /// Provides the output of MLC decision tree 0.
    Mlc0Src = 0x70,

    /// MLC source register 1.
    ///
    /// Provides the output of MLC decision tree 1.
    Mlc1Src = 0x71,

    /// MLC source register 2.
    ///
    /// Provides the output of MLC decision tree 2.
    Mlc2Src = 0x72,

    /// MLC source register 3.
    ///
    /// Provides the output of MLC decision tree 3.
    Mlc3Src = 0x73,

    /// MLC source register 4.
    ///
    /// Provides the output of MLC decision tree 4.
    Mlc4Src = 0x74,

    /// MLC source register 5.
    ///
    /// Provides the output of MLC decision tree 5.
    Mlc5Src = 0x75,

    /// MLC source register 6.
    ///
    /// Provides the output of MLC decision tree 6.
    Mlc6Src = 0x76,

    /// MLC source register 7.
    ///
    /// Provides the output of MLC decision tree 7.
    Mlc7Src = 0x77,
}

/// Page selection register (R/W).
///
/// The `PAGE_SEL` register is used to select the memory page for advanced configuration.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::PageSel, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PageSel {
    #[bits(4, access = RO, default = 0b0001)]
    not_used_01: u8,
    #[bits(4, default = 0)]
    pub page_sel: u8,
}

/// Embedded function enable register A (R/W).
///
/// The `EMB_FUNC_EN_A` register is used to enable embedded functions such as pedometer, tilt detection, and significant motion detection.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncEnA, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncEnA {
    #[bits(3, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub pedo_en: u8,
    #[bits(1, default = 0)]
    pub tilt_en: u8,
    #[bits(1, default = 0)]
    pub sign_motion_en: u8,
    #[bits(2, access = RO, default = 0)]
    not_used_02: u8,
}

/// Embedded function enable register B (R/W).
///
/// The `EMB_FUNC_EN_B` register is used to enable advanced features such as finite state machine (FSM), machine learning core (MLC), and FIFO compression.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncEnB, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncEnB {
    #[bits(1, default = 0)]
    pub fsm_en: u8,
    #[bits(2, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub fifo_compr_en: u8,
    #[bits(1, default = 0)]
    pub mlc_en: u8,
    #[bits(3, access = RO, default = 0)]
    not_used_02: u8,
}

/// Embedded function interrupt 1 register (R/W).
///
/// The `EMB_FUNC_INT1` register is used to configure interrupt routing to the INT1 pin for embedded functions such as step detection, tilt detection, and FSM long counter.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncInt1, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncInt1 {
    #[bits(3, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub int1_step_detector: u8,
    #[bits(1, default = 0)]
    pub int1_tilt: u8,
    #[bits(1, default = 0)]
    pub int1_sig_mot: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_02: u8,
    #[bits(1, default = 0)]
    pub int1_fsm_lc: u8,
}

/// Page address register (R/W).
///
/// The `PAGE_ADDRESS` register is used to specify the address of the memory page for advanced configuration.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::PageAddress, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PageAddress {
    #[bits(8, default = 0)]
    pub page_addr: u8,
}

/// Page value register (R/W).
///
/// The `PAGE_VALUE` register is used to read or write the value at the specified page address in the memory.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::PageValue, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PageValue {
    #[bits(8, default = 0)]
    pub page_value: u8,
}

/// FSM interrupt 1 register A (R/W).
///
/// The `FSM_INT1_A` register is used to configure interrupt routing to the INT1 pin for FSM programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmInt1A, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmInt1A {
    #[bits(1, default = 0)]
    pub int1_fsm1: u8,
    #[bits(1, default = 0)]
    pub int1_fsm2: u8,
    #[bits(1, default = 0)]
    pub int1_fsm3: u8,
    #[bits(1, default = 0)]
    pub int1_fsm4: u8,
    #[bits(1, default = 0)]
    pub int1_fsm5: u8,
    #[bits(1, default = 0)]
    pub int1_fsm6: u8,
    #[bits(1, default = 0)]
    pub int1_fsm7: u8,
    #[bits(1, default = 0)]
    pub int1_fsm8: u8,
}

/// FSM interrupt 1 register B (R/W).
///
/// The `FSM_INT1_B` register is used to configure interrupt routing to the INT1 pin for FSM programs 9 through 16.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmInt1B, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmInt1B {
    #[bits(1, default = 0)]
    pub int1_fsm9: u8,
    #[bits(1, default = 0)]
    pub int1_fsm10: u8,
    #[bits(1, default = 0)]
    pub int1_fsm11: u8,
    #[bits(1, default = 0)]
    pub int1_fsm12: u8,
    #[bits(1, default = 0)]
    pub int1_fsm13: u8,
    #[bits(1, default = 0)]
    pub int1_fsm14: u8,
    #[bits(1, default = 0)]
    pub int1_fsm15: u8,
    #[bits(1, default = 0)]
    pub int1_fsm16: u8,
}

/// MLC interrupt 1 register (R/W).
///
/// The `MLC_INT1` register is used to configure interrupt routing to the INT1 pin for MLC programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::MlcInt1, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcInt1 {
    #[bits(1, default = 0)]
    pub int1_mlc1: u8,
    #[bits(1, default = 0)]
    pub int1_mlc2: u8,
    #[bits(1, default = 0)]
    pub int1_mlc3: u8,
    #[bits(1, default = 0)]
    pub int1_mlc4: u8,
    #[bits(1, default = 0)]
    pub int1_mlc5: u8,
    #[bits(1, default = 0)]
    pub int1_mlc6: u8,
    #[bits(1, default = 0)]
    pub int1_mlc7: u8,
    #[bits(1, default = 0)]
    pub int1_mlc8: u8,
}

/// Embedded function interrupt 2 register (R/W).
///
/// The `EMB_FUNC_INT2` register is used to configure interrupt routing to the INT2 pin for embedded functions such as step detection, tilt detection, and FSM long counter.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncInt2, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncInt2 {
    #[bits(3, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub int2_step_detector: u8,
    #[bits(1, default = 0)]
    pub int2_tilt: u8,
    #[bits(1, default = 0)]
    pub int2_sig_mot: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_02: u8,
    #[bits(1, default = 0)]
    pub int2_fsm_lc: u8,
}

/// FSM interrupt 2 register A (R/W).
///
/// The `FSM_INT2_A` register is used to configure interrupt routing to the INT2 pin for FSM programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmInt2A, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmInt2A {
    #[bits(1, default = 0)]
    pub int2_fsm1: u8,
    #[bits(1, default = 0)]
    pub int2_fsm2: u8,
    #[bits(1, default = 0)]
    pub int2_fsm3: u8,
    #[bits(1, default = 0)]
    pub int2_fsm4: u8,
    #[bits(1, default = 0)]
    pub int2_fsm5: u8,
    #[bits(1, default = 0)]
    pub int2_fsm6: u8,
    #[bits(1, default = 0)]
    pub int2_fsm7: u8,
    #[bits(1, default = 0)]
    pub int2_fsm8: u8,
}

/// FSM interrupt 2 register B (R/W).
///
/// The `FSM_INT2_B` register is used to configure interrupt routing to the INT2 pin for FSM programs 9 through 16.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmInt2B, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmInt2B {
    #[bits(1, default = 0)]
    pub int2_fsm9: u8,
    #[bits(1, default = 0)]
    pub int2_fsm10: u8,
    #[bits(1, default = 0)]
    pub int2_fsm11: u8,
    #[bits(1, default = 0)]
    pub int2_fsm12: u8,
    #[bits(1, default = 0)]
    pub int2_fsm13: u8,
    #[bits(1, default = 0)]
    pub int2_fsm14: u8,
    #[bits(1, default = 0)]
    pub int2_fsm15: u8,
    #[bits(1, default = 0)]
    pub int2_fsm16: u8,
}

/// MLC interrupt 2 register (R/W).
///
/// The `MLC_INT2` register is used to configure interrupt routing to the INT2 pin for MLC programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::MlcInt2, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcInt2 {
    #[bits(1, default = 0)]
    pub int2_mlc1: u8,
    #[bits(1, default = 0)]
    pub int2_mlc2: u8,
    #[bits(1, default = 0)]
    pub int2_mlc3: u8,
    #[bits(1, default = 0)]
    pub int2_mlc4: u8,
    #[bits(1, default = 0)]
    pub int2_mlc5: u8,
    #[bits(1, default = 0)]
    pub int2_mlc6: u8,
    #[bits(1, default = 0)]
    pub int2_mlc7: u8,
    #[bits(1, default = 0)]
    pub int2_mlc8: u8,
}

/// Embedded function status register (R).
///
/// The `EMB_FUNC_STATUS` register provides the status of various embedded function events, such as step detection, tilt detection, and significant motion detection.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncStatus, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncStatus {
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

/// FSM status register A (R).
///
/// The `FSM_STATUS_A` register provides the status of FSM programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmStatusA, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmStatusA {
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

/// FSM status register B (R).
///
/// The `FSM_STATUS_B` register provides the status of FSM programs 9 through 16.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmStatusB, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmStatusB {
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

/// MLC status register (R).
///
/// The `MLC_STATUS` register provides the status of MLC programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::MlcStatus, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcStatus {
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

/// Page read/write register (R/W).
///
/// The `PAGE_RW` register is used to configure page read/write operations and embedded function latch interrupt request.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::PageRw, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct PageRw {
    #[bits(5, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(2, default = 0)]
    pub page_rw: u8,
    #[bits(1, default = 0)]
    pub emb_func_lir: u8,
}

/// Embedded function FIFO configuration register (R/W).
///
/// The `EMB_FUNC_FIFO_CFG` register is used to enable pedometer data storage in the FIFO.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncFifoCfg, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncFifoCfg {
    #[bits(6, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub pedo_fifo_en: u8,
    #[bits(1, access = RO, default = 0)]
    not_used_02: u8,
}

/// FSM enable register A (R/W).
///
/// The `FSM_ENABLE_A` register is used to enable FSM programs 1 through 8.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmEnableA, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmEnableA {
    #[bits(1, default = 0)]
    pub fsm1_en: u8,
    #[bits(1, default = 0)]
    pub fsm2_en: u8,
    #[bits(1, default = 0)]
    pub fsm3_en: u8,
    #[bits(1, default = 0)]
    pub fsm4_en: u8,
    #[bits(1, default = 0)]
    pub fsm5_en: u8,
    #[bits(1, default = 0)]
    pub fsm6_en: u8,
    #[bits(1, default = 0)]
    pub fsm7_en: u8,
    #[bits(1, default = 0)]
    pub fsm8_en: u8,
}

/// FSM enable register B (R/W).
///
/// The `FSM_ENABLE_B` register is used to enable FSM programs 9 through 16.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmEnableB, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmEnableB {
    #[bits(1, default = 0)]
    pub fsm9_en: u8,
    #[bits(1, default = 0)]
    pub fsm10_en: u8,
    #[bits(1, default = 0)]
    pub fsm11_en: u8,
    #[bits(1, default = 0)]
    pub fsm12_en: u8,
    #[bits(1, default = 0)]
    pub fsm13_en: u8,
    #[bits(1, default = 0)]
    pub fsm14_en: u8,
    #[bits(1, default = 0)]
    pub fsm15_en: u8,
    #[bits(1, default = 0)]
    pub fsm16_en: u8,
}

#[register(address = EmbReg::FsmLongCounterL, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct FsmLongCounter {
    #[bits(16, default = 0)]
    pub fsm_lc: u16,
}

/// FSM long counter clear register (R/W).
///
/// The `FSM_LONG_COUNTER_CLEAR` register is used to clear the FSM long counter.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmLongCounterClear, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmLongCounterClear {
    #[bits(2, default = 0)]
    pub fsm_lc_clr: u8,
    #[bits(6, access = RO, default = 0)]
    not_used_01: u8,
}

/// FSM output register 1-16 (R).
///
/// The `FSM_OUTS1-16` register provides the output status of FSM program 1-16.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::FsmOuts1, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FsmOutsReg {
    #[bits(1, access = RO)]
    pub n_v: u8,
    #[bits(1, access = RO)]
    pub p_v: u8,
    #[bits(1, access = RO)]
    pub n_z: u8,
    #[bits(1, access = RO)]
    pub p_z: u8,
    #[bits(1, access = RO)]
    pub n_y: u8,
    #[bits(1, access = RO)]
    pub p_y: u8,
    #[bits(1, access = RO)]
    pub n_x: u8,
    #[bits(1, access = RO)]
    pub p_x: u8,
}

/// Embedded function ODR configuration register B (R/W).
///
/// The `EMB_FUNC_ODR_CFG_B` register is used to configure the output data rate (ODR) for the finite state machine (FSM).
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncOdrCfgB, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncOdrCfgB {
    #[bits(3, access = RO, default = 0b011)]
    not_used_01: u8,
    #[bits(2, default = 0b01)]
    pub fsm_odr: u8,
    #[bits(3, access = RO, default = 0b010)]
    not_used_02: u8,
}

/// Embedded function ODR configuration register C (R/W).
///
/// The `EMB_FUNC_ODR_CFG_C` register is used to configure the output data rate (ODR) for the machine learning core (MLC).
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncOdrCfgC, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncOdrCfgC {
    #[bits(4, access = RO, default = 0b0101)]
    not_used_01: u8,
    #[bits(2, default = 0b01)]
    pub mlc_odr: u8,
    #[bits(2, access = RO, default = 0)]
    not_used_02: u8,
}

#[register(address = EmbReg::StepCounterL, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct StepCounter {
    #[bits(16, access = RO)]
    pub step: u16,
}

/// Embedded function source register (R/W).
///
/// The `EMB_FUNC_SRC` register provides information about step counter events and pedometer reset status.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncSrc, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncSrc {
    #[bits(2, access = RO)]
    not_used_01: u8,
    #[bits(1)]
    pub stepcounter_bit_set: u8,
    #[bits(1)]
    pub step_overflow: u8,
    #[bits(1)]
    pub step_count_delta_ia: u8,
    #[bits(1)]
    pub step_detected: u8,
    #[bits(1, access = RO)]
    not_used_02: u8,
    #[bits(1)]
    pub pedo_rst_step: u8,
}

/// Embedded function initialization register A (R/W).
///
/// The `EMB_FUNC_INIT_A` register is used to initialize embedded functions such as step detection, tilt detection, and significant motion detection.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncInitA, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncInitA {
    #[bits(3, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub step_det_init: u8,
    #[bits(1, default = 0)]
    pub tilt_init: u8,
    #[bits(1, default = 0)]
    pub sig_mot_init: u8,
    #[bits(2, access = RO, default = 0)]
    not_used_02: u8,
}

/// Embedded function initialization register B (R/W).
///
/// The `EMB_FUNC_INIT_B` register is used to initialize advanced features such as FSM, FIFO compression, and MLC.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::EmbFuncInitB, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct EmbFuncInitB {
    #[bits(1, default = 0)]
    pub fsm_init: u8,
    #[bits(2, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(1, default = 0)]
    pub fifo_compr_init: u8,
    #[bits(1, default = 0)]
    pub mlc_init: u8,
    #[bits(3, access = RO, default = 0)]
    not_used_02: u8,
}

/// MLC source register 0-7 (R).
///
/// The `MLC0_SRC-MLC7_SRC` register provides the output of MLC decision tree.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = EmbReg::Mlc0Src, access_type = "Ism330dhcx<B, T, EmbBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MlcSrcReg {
    #[bits(8, access = RO)]
    pub mlc_src: u8,
}

#[derive(Clone, Debug, Default)]
pub struct EmbFsmEnable {
    pub fsm_enable_a: FsmEnableA,
    pub fsm_enable_b: FsmEnableB,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum UncoptrRate {
    #[default]
    Disable = 0x00,
    Always = 0x04,
    _8To1 = 0x05,
    _16To1 = 0x06,
    _32To1 = 0x07,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsmLcClr {
    #[default]
    Normal = 0,
    Clear = 1,
    ClearDone = 2,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FsmOdr {
    _12_5hz = 0,
    #[default]
    _26hz = 1,
    _52hz = 2,
    _104hz = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum MlcOdr {
    _12_5hz = 0,
    #[default]
    _26hz = 1,
    _52hz = 2,
    _104hz = 3,
}
