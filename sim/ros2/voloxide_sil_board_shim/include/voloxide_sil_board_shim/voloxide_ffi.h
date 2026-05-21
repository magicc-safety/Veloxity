#ifndef VOLOXIDE_SIL_BOARD_SHIM_VOLOXIDE_FFI_H_
#define VOLOXIDE_SIL_BOARD_SHIM_VOLOXIDE_FFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VoloxideFfiHandle VoloxideFfiHandle;

typedef struct VoloxideFfiVector3
{
  double x;
  double y;
  double z;
} VoloxideFfiVector3;

typedef struct VoloxideFfiImu
{
  uint64_t timestamp_us;
  VoloxideFfiVector3 angular_velocity;
  VoloxideFfiVector3 linear_acceleration;
  float temperature_kelvin;
} VoloxideFfiImu;

typedef struct VoloxideFfiMag
{
  uint64_t timestamp_us;
  VoloxideFfiVector3 magnetic_field;
} VoloxideFfiMag;

typedef struct VoloxideFfiBaro
{
  uint64_t timestamp_us;
  float altitude;
  float pressure;
  float temperature_kelvin;
} VoloxideFfiBaro;

typedef struct VoloxideFfiGnss
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
} VoloxideFfiGnss;

typedef struct VoloxideFfiAirspeed
{
  uint64_t timestamp_us;
  float differential_pressure;
  float temperature_kelvin;
  float indicated_airspeed;
} VoloxideFfiAirspeed;

typedef struct VoloxideFfiRange
{
  uint64_t timestamp_us;
  float range;
  float min_range;
  float max_range;
} VoloxideFfiRange;

typedef struct VoloxideFfiBattery
{
  uint64_t timestamp_us;
  float voltage;
  float current;
} VoloxideFfiBattery;

typedef struct VoloxideFfiRc
{
  uint64_t timestamp_us;
  uint16_t values[8];
} VoloxideFfiRc;

typedef struct VoloxideFfiSensorSnapshot
{
  bool has_imu;
  VoloxideFfiImu imu;
  bool has_mag;
  VoloxideFfiMag mag;
  bool has_baro;
  VoloxideFfiBaro baro;
  bool has_gnss;
  VoloxideFfiGnss gnss;
  bool has_airspeed;
  VoloxideFfiAirspeed airspeed;
  bool has_range;
  VoloxideFfiRange range;
  bool has_battery;
  VoloxideFfiBattery battery;
  bool has_rc;
  VoloxideFfiRc rc;
} VoloxideFfiSensorSnapshot;

VoloxideFfiHandle * voloxide_sim_create(void);
void voloxide_sim_destroy(VoloxideFfiHandle * handle);
bool voloxide_sim_set_sensors(
  VoloxideFfiHandle * handle,
  const VoloxideFfiSensorSnapshot * snapshot);
bool voloxide_sim_run_once(VoloxideFfiHandle * handle);
size_t voloxide_sim_get_pwm(
  const VoloxideFfiHandle * handle,
  uint16_t * output,
  size_t output_len);

#ifdef __cplusplus
}
#endif

#endif  // VOLOXIDE_SIL_BOARD_SHIM_VOLOXIDE_FFI_H_
