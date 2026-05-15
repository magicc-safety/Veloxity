#ifndef RUST_SIL_BOARD_SHIM_RUSTFLIGHT_FFI_H_
#define RUST_SIL_BOARD_SHIM_RUSTFLIGHT_FFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RustflightFfiHandle RustflightFfiHandle;

typedef struct RustflightFfiVector3
{
  double x;
  double y;
  double z;
} RustflightFfiVector3;

typedef struct RustflightFfiImu
{
  uint64_t timestamp_us;
  RustflightFfiVector3 angular_velocity;
  RustflightFfiVector3 linear_acceleration;
  float temperature_kelvin;
} RustflightFfiImu;

typedef struct RustflightFfiMag
{
  uint64_t timestamp_us;
  RustflightFfiVector3 magnetic_field;
} RustflightFfiMag;

typedef struct RustflightFfiBaro
{
  uint64_t timestamp_us;
  float altitude;
  float pressure;
  float temperature_kelvin;
} RustflightFfiBaro;

typedef struct RustflightFfiGnss
{
  uint64_t timestamp_us;
  uint8_t fix_type;
  uint8_t num_sat;
  double lat_degrees;
  double lon_degrees;
  float alt;
  float horizontal_accuracy;
  float vertical_accuracy;
  float vel_n;
  float vel_e;
  float vel_d;
  float speed_accuracy;
  int64_t unix_seconds;
  int32_t unix_nanos;
} RustflightFfiGnss;

typedef struct RustflightFfiAirspeed
{
  uint64_t timestamp_us;
  float differential_pressure;
  float temperature_kelvin;
  float indicated_airspeed;
} RustflightFfiAirspeed;

typedef struct RustflightFfiRange
{
  uint64_t timestamp_us;
  float range;
  float min_range;
  float max_range;
} RustflightFfiRange;

typedef struct RustflightFfiBattery
{
  uint64_t timestamp_us;
  float voltage;
  float current;
} RustflightFfiBattery;

typedef struct RustflightFfiRc
{
  uint64_t timestamp_us;
  uint16_t values[8];
} RustflightFfiRc;

typedef struct RustflightFfiSensorSnapshot
{
  bool has_imu;
  RustflightFfiImu imu;
  bool has_mag;
  RustflightFfiMag mag;
  bool has_baro;
  RustflightFfiBaro baro;
  bool has_gnss;
  RustflightFfiGnss gnss;
  bool has_airspeed;
  RustflightFfiAirspeed airspeed;
  bool has_range;
  RustflightFfiRange range;
  bool has_battery;
  RustflightFfiBattery battery;
  bool has_rc;
  RustflightFfiRc rc;
} RustflightFfiSensorSnapshot;

RustflightFfiHandle * rustflight_sim_create(void);
void rustflight_sim_destroy(RustflightFfiHandle * handle);
bool rustflight_sim_set_sensors(
  RustflightFfiHandle * handle,
  const RustflightFfiSensorSnapshot * snapshot);
bool rustflight_sim_run_once(RustflightFfiHandle * handle);
size_t rustflight_sim_get_pwm(
  const RustflightFfiHandle * handle,
  uint16_t * output,
  size_t output_len);

#ifdef __cplusplus
}
#endif

#endif  // RUST_SIL_BOARD_SHIM_RUSTFLIGHT_FFI_H_
