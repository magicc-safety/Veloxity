#ifndef VELOXITY_SIL_BOARD_SHIM_VELOXITY_FFI_H_
#define VELOXITY_SIL_BOARD_SHIM_VELOXITY_FFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VeloxityFfiHandle VeloxityFfiHandle;

typedef struct VeloxityFfiVector3
{
  double x;
  double y;
  double z;
} VeloxityFfiVector3;

typedef struct VeloxityFfiImu
{
  uint64_t timestamp_us;
  VeloxityFfiVector3 angular_velocity;
  VeloxityFfiVector3 linear_acceleration;
  float temperature_kelvin;
} VeloxityFfiImu;

typedef struct VeloxityFfiMag
{
  uint64_t timestamp_us;
  VeloxityFfiVector3 magnetic_field;
} VeloxityFfiMag;

typedef struct VeloxityFfiBaro
{
  uint64_t timestamp_us;
  float altitude;
  float pressure;
  float temperature_kelvin;
} VeloxityFfiBaro;

typedef struct VeloxityFfiGnss
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
} VeloxityFfiGnss;

typedef struct VeloxityFfiAirspeed
{
  uint64_t timestamp_us;
  float differential_pressure;
  float temperature_kelvin;
  float indicated_airspeed;
} VeloxityFfiAirspeed;

typedef struct VeloxityFfiRange
{
  uint64_t timestamp_us;
  float range;
  float min_range;
  float max_range;
} VeloxityFfiRange;

typedef struct VeloxityFfiBattery
{
  uint64_t timestamp_us;
  float voltage;
  float current;
} VeloxityFfiBattery;

typedef struct VeloxityFfiRc
{
  uint64_t timestamp_us;
  uint16_t values[8];
} VeloxityFfiRc;

typedef struct VeloxityFfiSensorSnapshot
{
  bool has_imu;
  VeloxityFfiImu imu;
  bool has_mag;
  VeloxityFfiMag mag;
  bool has_baro;
  VeloxityFfiBaro baro;
  bool has_gnss;
  VeloxityFfiGnss gnss;
  bool has_airspeed;
  VeloxityFfiAirspeed airspeed;
  bool has_range;
  VeloxityFfiRange range;
  bool has_battery;
  VeloxityFfiBattery battery;
  bool has_rc;
  VeloxityFfiRc rc;
} VeloxityFfiSensorSnapshot;

VeloxityFfiHandle * veloxity_sim_create(void);
void veloxity_sim_destroy(VeloxityFfiHandle * handle);
bool veloxity_sim_set_sensors(
  const VeloxityFfiHandle * handle,
  const VeloxityFfiSensorSnapshot * snapshot);
bool veloxity_sim_sync_latest_imu(const VeloxityFfiHandle * handle);
uint64_t veloxity_sim_clock_micros(const VeloxityFfiHandle * handle);
size_t veloxity_sim_get_pwm(
  const VeloxityFfiHandle * handle,
  uint16_t * output,
  size_t output_len);

#ifdef __cplusplus
}
#endif

#endif  // VELOXITY_SIL_BOARD_SHIM_VELOXITY_FFI_H_
