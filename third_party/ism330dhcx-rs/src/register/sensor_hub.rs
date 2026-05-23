use super::super::{
    BusOperation, DelayNs, Error, Ism330dhcx, RegisterOperation, SensorOperation, bisync,
    register::SensHubBank,
};

use bitfield_struct::bitfield;
use derive_more::TryFrom;
use st_mem_bank_macro::register;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum SensHubReg {
    /// Sensor hub output register 1.
    ///
    /// Stores the first byte of data read from external sensors.
    SensorHub1 = 0x02,

    /// Sensor hub output register 2.
    ///
    /// Stores the second byte of data read from external sensors.
    SensorHub2 = 0x03,

    /// Sensor hub output register 3.
    ///
    /// Stores the third byte of data read from external sensors.
    SensorHub3 = 0x04,

    /// Sensor hub output register 4.
    ///
    /// Stores the fourth byte of data read from external sensors.
    SensorHub4 = 0x05,

    /// Sensor hub output register 5.
    ///
    /// Stores the fifth byte of data read from external sensors.
    SensorHub5 = 0x06,

    /// Sensor hub output register 6.
    ///
    /// Stores the sixth byte of data read from external sensors.
    SensorHub6 = 0x07,

    /// Sensor hub output register 7.
    ///
    /// Stores the seventh byte of data read from external sensors.
    SensorHub7 = 0x08,

    /// Sensor hub output register 8.
    ///
    /// Stores the eighth byte of data read from external sensors.
    SensorHub8 = 0x09,

    /// Sensor hub output register 9.
    ///
    /// Stores the ninth byte of data read from external sensors.
    SensorHub9 = 0x0A,

    /// Sensor hub output register 10.
    ///
    /// Stores the tenth byte of data read from external sensors.
    SensorHub10 = 0x0B,

    /// Sensor hub output register 11.
    ///
    /// Stores the eleventh byte of data read from external sensors.
    SensorHub11 = 0x0C,

    /// Sensor hub output register 12.
    ///
    /// Stores the twelfth byte of data read from external sensors.
    SensorHub12 = 0x0D,

    /// Sensor hub output register 13.
    ///
    /// Stores the thirteenth byte of data read from external sensors.
    SensorHub13 = 0x0E,

    /// Sensor hub output register 14.
    ///
    /// Stores the fourteenth byte of data read from external sensors.
    SensorHub14 = 0x0F,

    /// Sensor hub output register 15.
    ///
    /// Stores the fifteenth byte of data read from external sensors.
    SensorHub15 = 0x10,

    /// Sensor hub output register 16.
    ///
    /// Stores the sixteenth byte of data read from external sensors.
    SensorHub16 = 0x11,

    /// Sensor hub output register 17.
    ///
    /// Stores the seventeenth byte of data read from external sensors.
    SensorHub17 = 0x12,

    /// Sensor hub output register 18.
    ///
    /// Stores the eighteenth byte of data read from external sensors.
    SensorHub18 = 0x13,

    /// Master configuration register.
    ///
    /// Configures the sensor hub and its operational modes.
    MasterConfig = 0x14,

    /// Slave 0 address register.
    ///
    /// Specifies the I²C address of the first external sensor.
    Slv0Add = 0x15,

    /// Slave 0 sub-address register.
    ///
    /// Specifies the register address within the first external sensor.
    Slv0Subadd = 0x16,

    /// Slave 0 configuration register.
    ///
    /// Configures read/write operations for the first external sensor.
    Slv0Config = 0x17,

    /// Slave 1 address register.
    ///
    /// Specifies the I²C address of the second external sensor.
    Slv1Add = 0x18,

    /// Slave 1 sub-address register.
    ///
    /// Specifies the register address within the second external sensor.
    Slv1Subadd = 0x19,

    /// Slave 1 configuration register.
    ///
    /// Configures read/write operations for the second external sensor.
    Slv1Config = 0x1A,

    /// Slave 2 address register.
    ///
    /// Specifies the I²C address of the third external sensor.
    Slv2Add = 0x1B,

    /// Slave 2 sub-address register.
    ///
    /// Specifies the register address within the third external sensor.
    Slv2Subadd = 0x1C,

    /// Slave 2 configuration register.
    ///
    /// Configures read/write operations for the third external sensor.
    Slv2Config = 0x1D,

    /// Slave 3 address register.
    ///
    /// Specifies the I²C address of the fourth external sensor.
    Slv3Add = 0x1E,

    /// Slave 3 sub-address register.
    ///
    /// Specifies the register address within the fourth external sensor.
    Slv3Subadd = 0x1F,

    /// Slave 3 configuration register.
    ///
    /// Configures read/write operations for the fourth external sensor.
    Slv3Config = 0x20,

    /// Data write register for slave 0.
    ///
    /// Holds the data to be written to the first external sensor.
    DatawriteSlv0 = 0x21,

    /// Sensor hub status register.
    ///
    /// Provides the status of sensor hub operations, including errors.
    StatusMaster = 0x22,
}

/// Sensor hub register 1-18 (R).
///
/// The `SENSOR_HUB1` register provides data from the first external sensor connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::SensorHub1, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct SensorHubReg {
    #[bits(8, access = RO)]
    pub sensor_hub: u8,
}

/// Master configuration register (R/W).
///
/// The `MASTER_CONFIG` register is used to configure the sensor hub master settings, such as enabling the master, pass-through mode, and resetting master registers.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::MasterConfig, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct MasterConfig {
    #[bits(2, default = 0)]
    pub aux_sens_on: u8,
    #[bits(1, default = 0)]
    pub master_on: u8,
    #[bits(1, default = 0)]
    pub shub_pu_en: u8,
    #[bits(1, default = 0)]
    pub pass_through_mode: u8,
    #[bits(1, default = 0)]
    pub start_config: u8,
    #[bits(1, default = 0)]
    pub write_once: u8,
    #[bits(1, default = 0)]
    pub rst_master_regs: u8,
}

/// Slave 0 address register (R/W).
///
/// The `SLV0_ADD` register is used to configure the address and read/write mode for the first slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv0Add, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv0Add {
    #[bits(1, default = 0)]
    pub rw_0: u8,
    #[bits(7, default = 0)]
    pub slave0: u8,
}

/// Slave 0 subaddress register (R/W).
///
/// The `SLV0_SUBADD` register is used to configure the subaddress (register address) for the first slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv0Subadd, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv0Subadd {
    #[bits(8, default = 0)]
    pub slave0_reg: u8,
}

/// Slave 0 configuration register (R/W).
///
/// The `SLV0_CONFIG` register is used to configure the number of operations, batch enable, and output data rate for the first slave device.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv0Config, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv0Config {
    #[bits(3, default = 0)]
    pub slave0_numop: u8,
    #[bits(1, default = 0)]
    pub batch_ext_sens_0_en: u8,
    #[bits(2, access = RO, default = 0)]
    not_used_01: u8,
    #[bits(2, default = 0)]
    pub shub_odr: u8,
}

/// Slave 1 address register (R/W).
///
/// The `SLV1_ADD` register is used to configure the address and read/write mode for the second slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv1Add, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv1Add {
    #[bits(1, default = 0)]
    pub r_1: u8,
    #[bits(7, default = 0)]
    pub slave1_add: u8,
}

/// Slave 1 subaddress register (R/W).
///
/// The `SLV1_SUBADD` register is used to configure the subaddress (register address) for the second slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv1Subadd, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv1Subadd {
    #[bits(8, default = 0)]
    pub slave1_reg: u8,
}

/// Slave 1 configuration register (R/W).
///
/// The `SLV1_CONFIG` register is used to configure the number of operations and batch enable for the second slave device.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv1Config, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv1Config {
    #[bits(3, default = 0)]
    pub slave1_numop: u8,
    #[bits(1, default = 0)]
    pub batch_ext_sens_1_en: u8,
    #[bits(4, access = RO, default = 0)]
    not_used_01: u8,
}

/// Slave 2 address register (R/W).
///
/// The `SLV2_ADD` register is used to configure the address and read/write mode for the third slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv2Add, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv2Add {
    #[bits(1, default = 0)]
    pub r_2: u8,
    #[bits(7, default = 0)]
    pub slave2_add: u8,
}

/// Slave 2 subaddress register (R/W).
///
/// The `SLV2_SUBADD` register is used to configure the subaddress (register address) for the third slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv2Subadd, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv2Subadd {
    #[bits(8, default = 0)]
    pub slave2_reg: u8,
}

/// Slave 2 configuration register (R/W).
///
/// The `SLV2_CONFIG` register is used to configure the number of operations and batch enable for the third slave device.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv2Config, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv2Config {
    #[bits(3, default = 0)]
    pub slave2_numop: u8,
    #[bits(1, default = 0)]
    pub batch_ext_sens_2_en: u8,
    #[bits(4, access = RO, default = 0)]
    not_used_01: u8,
}

/// Slave 3 address register (R/W).
///
/// The `SLV3_ADD` register is used to configure the address and read/write mode for the fourth slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv3Add, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv3Add {
    #[bits(1, default = 0)]
    pub r_3: u8,
    #[bits(7, default = 0)]
    pub slave3_add: u8,
}

/// Slave 3 subaddress register (R/W).
///
/// The `SLV3_SUBADD` register is used to configure the subaddress (register address) for the fourth slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv3Subadd, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv3Subadd {
    #[bits(8, default = 0)]
    pub slave3_reg: u8,
}

/// Slave 3 configuration register (R/W).
///
/// The `SLV3_CONFIG` register is used to configure the number of operations and batch enable for the fourth slave device.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::Slv3Config, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Slv3Config {
    #[bits(3, default = 0)]
    pub slave3_numop: u8,
    #[bits(1, default = 0)]
    pub batch_ext_sens_3_en: u8,
    #[bits(4, access = RO, default = 0)]
    not_used_01: u8,
}

/// Data write register for slave 0 (R/W).
///
/// The `DATAWRITE_SLV0` register is used to write data to the first slave device connected to the sensor hub.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::DatawriteSlv0, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct DatawriteSlv0 {
    #[bits(8, default = 0)]
    pub slave0_dataw: u8,
}

/// Master status register (R).
///
/// The `STATUS_MASTER` register provides the status of the sensor hub master, including operation completion and NACK errors for slave devices.
///
/// The bit order for this struct can be configured using the `bit_order_msb` feature:
/// * `Msb`: Most significant bit first.
/// * `Lsb`: Least significant bit first (default).
#[register(address = SensHubReg::StatusMaster, access_type = "Ism330dhcx<B, T, SensHubBank>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusMaster {
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

pub struct ShCfgWrite {
    pub slv0_add: u8,
    pub slv0_subadd: u8,
    pub slv0_data: u8,
}

pub struct ShCfgRead {
    pub slv_add: u8,
    pub slv_subadd: u8,
    pub slv_len: u8,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum AuxSensOn {
    #[default]
    Slv0 = 0,
    Slv01 = 1,
    Slv012 = 2,
    Slv0123 = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ShubPuEn {
    #[default]
    InternalPullUpOff = 0,
    InternalPullUpOn = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum StartConfig {
    #[default]
    ExtOnInt2Pin = 1,
    XlGyDrdy = 0,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum WriteOnce {
    #[default]
    EachShCycle = 0,
    OnlyFirstCycle = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum ShubOdr {
    #[default]
    _104hz = 0,
    _52hz = 1,
    _26hz = 2,
    _12_5hz = 3,
}
