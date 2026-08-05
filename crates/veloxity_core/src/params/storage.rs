//! ROSflight 2.0-compatible nonvolatile parameter image codec.

use super::{PARAM_DEFINITIONS, PARAMS_COUNT, ParamDefinition, ParamId, ParamValue, Params};

/// Byte size of ROSflight 2.0's ARM C++ `params_t`.
pub const ROSFLIGHT_C_PARAM_STORAGE_SIZE: usize = 7004;

// This is the eight-digit hash compiled into current upstream C main
// (a46527bd). C rejects parameter images from any other firmware hash.
const ROSFLIGHT_C_VERSION: u32 = 0xA465_27BD;
const ROSFLIGHT_C_PARAM_COUNT: usize = 333;
const ROSFLIGHT_C_PARAM_NAME_SIZE: usize = 16;
const PARAM_STORAGE_VALUE_SIZE: usize = size_of::<u32>();
// ARM's C++ ABI stores ROSflight's three-variant param_type_t in one byte.
const PARAM_STORAGE_TYPE_SIZE: usize = size_of::<u8>();
const PARAM_STORAGE_VALUES_OFFSET: usize = 8;
const PARAM_STORAGE_NAMES_OFFSET: usize =
    PARAM_STORAGE_VALUES_OFFSET + ROSFLIGHT_C_PARAM_COUNT * PARAM_STORAGE_VALUE_SIZE;
const PARAM_STORAGE_TYPES_OFFSET: usize =
    PARAM_STORAGE_NAMES_OFFSET + ROSFLIGHT_C_PARAM_COUNT * ROSFLIGHT_C_PARAM_NAME_SIZE;
const PARAM_STORAGE_MAGIC_EF_OFFSET: usize =
    PARAM_STORAGE_TYPES_OFFSET + ROSFLIGHT_C_PARAM_COUNT * PARAM_STORAGE_TYPE_SIZE;
const PARAM_STORAGE_CHECKSUM_OFFSET: usize = PARAM_STORAGE_MAGIC_EF_OFFSET + 1;
const RUST_ONLY_PARAM_COUNT: usize = 16;

const _: () = assert!(PARAMS_COUNT - RUST_ONLY_PARAM_COUNT == ROSFLIGHT_C_PARAM_COUNT);
const _: () = assert!(PARAM_STORAGE_CHECKSUM_OFFSET < ROSFLIGHT_C_PARAM_STORAGE_SIZE);

fn is_rosflight_c_param(id: ParamId) -> bool {
    !matches!(
        id,
        ParamId::PARAM_CHANNEL_OUTPUT_MASK
            | ParamId::PARAM_ALLOW_UNHEALTHY_ESTIMATOR
            | ParamId::PARAM_EST_ANGLE_LOCKOUT
            | ParamId::PARAM_RC_OUTPUT_KILL_CHANNEL
            | ParamId::PARAM_TELEM_HEARTBEAT_HZ
            | ParamId::PARAM_TELEM_STATUS_HZ
            | ParamId::PARAM_TELEM_IMU_HZ
            | ParamId::PARAM_TELEM_ATTITUDE_HZ
            | ParamId::PARAM_TELEM_OUTPUT_RAW_HZ
            | ParamId::PARAM_TELEM_DIFF_PRESSURE_HZ
            | ParamId::PARAM_TELEM_BARO_HZ
            | ParamId::PARAM_TELEM_MAG_HZ
            | ParamId::PARAM_TELEM_RANGE_HZ
            | ParamId::PARAM_TELEM_BATTERY_HZ
            | ParamId::PARAM_TELEM_GNSS_HZ
            | ParamId::PARAM_TELEM_RC_HZ
    )
}

fn rosflight_c_param_type(value: ParamValue) -> Option<u8> {
    match value {
        ParamValue::Int(_) => Some(0),
        ParamValue::Float(_) => Some(1),
        ParamValue::Uint(_) | ParamValue::Bool(_) => None,
    }
}

fn rosflight_c_params() -> impl Iterator<Item = &'static ParamDefinition> {
    PARAM_DEFINITIONS
        .iter()
        .filter(|definition| is_rosflight_c_param(definition.id))
}

fn rosflight_c_checksum(bytes: &[u8; ROSFLIGHT_C_PARAM_STORAGE_SIZE]) -> u8 {
    bytes[PARAM_STORAGE_VALUES_OFFSET..PARAM_STORAGE_MAGIC_EF_OFFSET]
        .iter()
        .fold(0, |checksum, byte| checksum ^ byte)
}

fn encode_param_value(value: ParamValue, definition: &ParamDefinition) -> Option<[u8; 4]> {
    match (value, definition.default) {
        (ParamValue::Float(value), ParamValue::Float(_)) => Some(value.to_bits().to_le_bytes()),
        (ParamValue::Int(value), ParamValue::Int(_)) => Some(value.to_le_bytes()),
        _ => None,
    }
}

fn decode_param_value(bytes: [u8; 4], definition: &ParamDefinition) -> Option<ParamValue> {
    Some(match definition.default {
        ParamValue::Float(_) => ParamValue::Float(f32::from_bits(u32::from_le_bytes(bytes))),
        ParamValue::Int(_) => ParamValue::Int(i32::from_le_bytes(bytes)),
        ParamValue::Uint(_) | ParamValue::Bool(_) => return None,
    })
}

/// Encodes the C-compatible subset into ROSflight 2.0's persisted `params_t`.
pub fn encode_rosflight_c_params(
    params: &Params,
    bytes: &mut [u8; ROSFLIGHT_C_PARAM_STORAGE_SIZE],
) -> bool {
    bytes.fill(0);
    bytes[..4].copy_from_slice(&ROSFLIGHT_C_VERSION.to_le_bytes());
    bytes[4..6].copy_from_slice(&(ROSFLIGHT_C_PARAM_STORAGE_SIZE as u16).to_le_bytes());
    bytes[6] = 0xBE;

    let mut count = 0;
    for (index, definition) in rosflight_c_params().enumerate() {
        count += 1;
        let offset = PARAM_STORAGE_VALUES_OFFSET + index * PARAM_STORAGE_VALUE_SIZE;
        let Some(encoded) = encode_param_value(params.get_by_id(definition.id), definition) else {
            return false;
        };
        bytes[offset..offset + PARAM_STORAGE_VALUE_SIZE].copy_from_slice(&encoded);

        let name_offset = PARAM_STORAGE_NAMES_OFFSET + index * ROSFLIGHT_C_PARAM_NAME_SIZE;
        let name = definition.name.as_bytes();
        if name.len() > ROSFLIGHT_C_PARAM_NAME_SIZE {
            return false;
        }
        bytes[name_offset..name_offset + name.len()].copy_from_slice(name);

        let type_offset = PARAM_STORAGE_TYPES_OFFSET + index * PARAM_STORAGE_TYPE_SIZE;
        let Some(param_type) = rosflight_c_param_type(definition.default) else {
            return false;
        };
        bytes[type_offset] = param_type;
    }
    if count != ROSFLIGHT_C_PARAM_COUNT {
        return false;
    }

    bytes[PARAM_STORAGE_MAGIC_EF_OFFSET] = 0xEF;
    bytes[PARAM_STORAGE_CHECKSUM_OFFSET] = rosflight_c_checksum(bytes);
    true
}

/// Decodes ROSflight 2.0's persisted `params_t` into the Rust parameter table.
/// Rust-only parameters retain their defaults because they are absent on disk.
pub fn decode_rosflight_c_params(bytes: &[u8; ROSFLIGHT_C_PARAM_STORAGE_SIZE]) -> Option<Params> {
    if u32::from_le_bytes(bytes[..4].try_into().ok()?) != ROSFLIGHT_C_VERSION
        || usize::from(u16::from_le_bytes(bytes[4..6].try_into().ok()?))
            != ROSFLIGHT_C_PARAM_STORAGE_SIZE
        || bytes[6] != 0xBE
        || bytes[PARAM_STORAGE_MAGIC_EF_OFFSET] != 0xEF
        || bytes[PARAM_STORAGE_CHECKSUM_OFFSET] != rosflight_c_checksum(bytes)
    {
        return None;
    }

    let mut params = Params::default();
    let mut count = 0;
    for (index, definition) in rosflight_c_params().enumerate() {
        count += 1;
        let name_offset = PARAM_STORAGE_NAMES_OFFSET + index * ROSFLIGHT_C_PARAM_NAME_SIZE;
        let stored_name = &bytes[name_offset..name_offset + ROSFLIGHT_C_PARAM_NAME_SIZE];
        let expected_name = definition.name.as_bytes();
        if stored_name[..expected_name.len()] != *expected_name
            || (expected_name.len() < ROSFLIGHT_C_PARAM_NAME_SIZE
                && stored_name[expected_name.len()] != 0)
        {
            return None;
        }

        let type_offset = PARAM_STORAGE_TYPES_OFFSET + index * PARAM_STORAGE_TYPE_SIZE;
        if bytes[type_offset] != rosflight_c_param_type(definition.default)? {
            return None;
        }

        let offset = PARAM_STORAGE_VALUES_OFFSET + index * PARAM_STORAGE_VALUE_SIZE;
        let value = decode_param_value(
            bytes[offset..offset + PARAM_STORAGE_VALUE_SIZE]
                .try_into()
                .ok()?,
            definition,
        )?;
        params.set_by_id(definition.id, value);
    }

    (count == ROSFLIGHT_C_PARAM_COUNT).then_some(params)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn storage_format_round_trips_only_c_parameters() {
        let mut params = Params::default();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(-0.25));
        params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_ALLOW_UNHEALTHY_ESTIMATOR, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_EST_ANGLE_LOCKOUT, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_RC_OUTPUT_KILL_CHANNEL, ParamValue::Int(4));
        params.set_by_id(ParamId::PARAM_TELEM_BARO_HZ, ParamValue::Int(20));

        let mut bytes = [0; ROSFLIGHT_C_PARAM_STORAGE_SIZE];
        assert!(encode_rosflight_c_params(&params, &mut bytes));
        let decoded = decode_rosflight_c_params(&bytes).expect("valid image must decode");

        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_GYRO_X_BIAS),
            ParamValue::Float(-0.25)
        );
        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK),
            ParamValue::Int(-1)
        );
        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_ALLOW_UNHEALTHY_ESTIMATOR),
            ParamValue::Int(1)
        );
        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_EST_ANGLE_LOCKOUT),
            ParamValue::Int(0)
        );
        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_RC_OUTPUT_KILL_CHANNEL),
            ParamValue::Int(-1)
        );
        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_TELEM_BARO_HZ),
            ParamValue::Int(100)
        );
    }

    #[test]
    fn storage_matches_rosflight_c_arm_layout() {
        let mut bytes = [0; ROSFLIGHT_C_PARAM_STORAGE_SIZE];
        assert!(encode_rosflight_c_params(&Params::default(), &mut bytes));

        assert_eq!(&bytes[..4], &ROSFLIGHT_C_VERSION.to_le_bytes());
        assert_eq!(
            &bytes[4..6],
            &(ROSFLIGHT_C_PARAM_STORAGE_SIZE as u16).to_le_bytes()
        );
        assert_eq!(bytes[6], 0xBE);
        assert_eq!(bytes[7], 0);
        assert_eq!(bytes[PARAM_STORAGE_MAGIC_EF_OFFSET], 0xEF);
        assert_eq!(
            bytes[PARAM_STORAGE_CHECKSUM_OFFSET],
            rosflight_c_checksum(&bytes)
        );
        assert_eq!(
            &bytes[PARAM_STORAGE_NAMES_OFFSET..PARAM_STORAGE_NAMES_OFFSET + 10],
            b"BAUD_RATE\0"
        );
        assert_eq!(bytes[PARAM_STORAGE_TYPES_OFFSET], 0);
    }

    #[test]
    fn storage_rejects_version_mismatch() {
        let mut bytes = [0; ROSFLIGHT_C_PARAM_STORAGE_SIZE];
        assert!(encode_rosflight_c_params(&Params::default(), &mut bytes));
        bytes[0] ^= 1;
        assert!(decode_rosflight_c_params(&bytes).is_none());
    }

    #[test]
    fn storage_rejects_schema_mismatch_even_with_valid_checksum() {
        let mut bytes = [0; ROSFLIGHT_C_PARAM_STORAGE_SIZE];
        assert!(encode_rosflight_c_params(&Params::default(), &mut bytes));
        bytes[PARAM_STORAGE_NAMES_OFFSET] ^= 1;
        bytes[PARAM_STORAGE_CHECKSUM_OFFSET] = rosflight_c_checksum(&bytes);
        assert!(decode_rosflight_c_params(&bytes).is_none());
    }
}
