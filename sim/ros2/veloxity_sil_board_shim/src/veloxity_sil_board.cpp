#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/airspeed.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/battery_status.hpp>
#include <rosflight_msgs/msg/gnss.hpp>
#include <rosflight_msgs/msg/pwm_output.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "veloxity_sil_board_shim/veloxity_ffi.h"

namespace veloxity_sil_board_shim
{
namespace
{
constexpr std::size_t kPwmChannelCount = 14;
constexpr uint16_t kDisabledPwmMicros = 1000;
constexpr uint16_t kRcLowMicros = 1000;
constexpr uint16_t kRcCenterMicros = 1500;
constexpr auto kExpectedSilRunPeriod = std::chrono::microseconds{2500};
constexpr auto kWarnSilRunGap = std::chrono::milliseconds{10};
constexpr auto kWarnSilRunDuration = std::chrono::milliseconds{4};

VeloxityFfiVector3 vector_to_ffi(const geometry_msgs::msg::Vector3 & vector)
{
  return VeloxityFfiVector3{vector.x, vector.y, vector.z};
}
}

class VeloxitySilBoard final : public rclcpp::Node
{
public:
  VeloxitySilBoard()
  : rclcpp::Node("veloxity_sil_board")
  {
    declare_parameter<std::string>("simulation_host", "localhost");
    declare_parameter<int>("simulation_port", 14525);
    declare_parameter<std::string>("ROS_host", "localhost");
    declare_parameter<int>("ROS_port", 14520);
    declare_parameter<int64_t>("serial_delay_ns", 6000000);

    pwm_outputs_.fill(kDisabledPwmMicros);
    firmware_.reset(veloxity_sim_create());
    if (!firmware_) {
      RCLCPP_ERROR(
        get_logger(),
        "failed to initialize Veloxity FFI; check VELOXITY_SIM_PARAM_DIR and MAVLink UDP port availability");
    }

    initialize_default_rc();
    submit_rc(latest_rc_);
    run_service_ = create_service<std_srvs::srv::Trigger>(
      "sil_board/run",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        response->success = synchronize_pwm();
        response->message = response->success ? "Veloxity SIL PWM synchronized" :
          "Veloxity SIL PWM synchronization failed";
      });

    pwm_publisher_ = create_publisher<rosflight_msgs::msg::PwmOutput>("sim/pwm_output", 1);

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "sim/sensors/imu/data", 1,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        VeloxityFfiSensorSnapshot snapshot{};
        snapshot.has_imu = true;
        snapshot.imu.timestamp_us = fcu_clock_micros();
        snapshot.imu.angular_velocity = vector_to_ffi(message->angular_velocity);
        snapshot.imu.linear_acceleration = vector_to_ffi(message->linear_acceleration);
        snapshot.imu.temperature_kelvin = imu_temperature_available_ ?
          static_cast<float>(latest_imu_temperature_.temperature) :
          298.15f;
        submit_snapshot(snapshot, "IMU");
      });

    imu_temperature_subscription_ = create_subscription<sensor_msgs::msg::Temperature>(
      "sim/sensors/imu/temperature", 1,
      [this](sensor_msgs::msg::Temperature::ConstSharedPtr message) {
        latest_imu_temperature_ = *message;
        imu_temperature_available_ = true;
      });

    mag_subscription_ = create_subscription<sensor_msgs::msg::MagneticField>(
      "sim/sensors/mag", 1,
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr message) {
        VeloxityFfiSensorSnapshot snapshot{};
        snapshot.has_mag = true;
        snapshot.mag.timestamp_us = fcu_clock_micros();
        snapshot.mag.magnetic_field = vector_to_ffi(message->magnetic_field);
        submit_snapshot(snapshot, "magnetometer");
      });

    baro_subscription_ = create_subscription<rosflight_msgs::msg::Barometer>(
      "sim/sensors/baro", 1,
      [this](rosflight_msgs::msg::Barometer::ConstSharedPtr message) {
        VeloxityFfiSensorSnapshot snapshot{};
        snapshot.has_baro = true;
        snapshot.baro.timestamp_us = fcu_clock_micros();
        snapshot.baro.altitude = message->altitude;
        snapshot.baro.pressure = message->pressure;
        snapshot.baro.temperature_kelvin = message->temperature;
        submit_snapshot(snapshot, "barometer");
      });

    gnss_subscription_ = create_subscription<rosflight_msgs::msg::GNSS>(
      "sim/sensors/gnss", 1,
      [this](rosflight_msgs::msg::GNSS::ConstSharedPtr message) {
        VeloxityFfiSensorSnapshot snapshot{};
        snapshot.has_gnss = true;
        snapshot.gnss.timestamp_us = fcu_clock_micros();
        snapshot.gnss.fix_type = message->fix_type;
        snapshot.gnss.num_sat = message->num_sat;
        snapshot.gnss.lat_degrees = message->lat;
        snapshot.gnss.lon_degrees = message->lon;
        snapshot.gnss.alt = message->alt;
        snapshot.gnss.horizontal_accuracy = message->horizontal_accuracy;
        snapshot.gnss.vertical_accuracy = message->vertical_accuracy;
        snapshot.gnss.vel_n = message->vel_n;
        snapshot.gnss.vel_e = message->vel_e;
        snapshot.gnss.vel_d = message->vel_d;
        snapshot.gnss.speed_accuracy = message->speed_accuracy;
        snapshot.gnss.unix_seconds = message->gnss_unix_seconds;
        snapshot.gnss.unix_nanos = message->gnss_unix_nanos;
        submit_snapshot(snapshot, "GNSS");
      });

    diff_pressure_subscription_ = create_subscription<rosflight_msgs::msg::Airspeed>(
      "sim/sensors/diff_pressure", 1,
      [this](rosflight_msgs::msg::Airspeed::ConstSharedPtr message) {
        VeloxityFfiSensorSnapshot snapshot{};
        snapshot.has_airspeed = true;
        snapshot.airspeed.timestamp_us = fcu_clock_micros();
        snapshot.airspeed.differential_pressure = message->differential_pressure;
        snapshot.airspeed.temperature_kelvin = message->temperature;
        snapshot.airspeed.indicated_airspeed = message->velocity;
        submit_snapshot(snapshot, "airspeed");
      });

    range_subscription_ = create_subscription<sensor_msgs::msg::Range>(
      "sim/sensors/range", 1,
      [this](sensor_msgs::msg::Range::ConstSharedPtr message) {
        VeloxityFfiSensorSnapshot snapshot{};
        snapshot.has_range = true;
        snapshot.range.timestamp_us = fcu_clock_micros();
        snapshot.range.range = message->range;
        snapshot.range.min_range = message->min_range;
        snapshot.range.max_range = message->max_range;
        submit_snapshot(snapshot, "range");
      });

    battery_subscription_ = create_subscription<rosflight_msgs::msg::BatteryStatus>(
      "sim/sensors/battery", 1,
      [this](rosflight_msgs::msg::BatteryStatus::ConstSharedPtr message) {
        VeloxityFfiSensorSnapshot snapshot{};
        snapshot.has_battery = true;
        snapshot.battery.timestamp_us = fcu_clock_micros();
        snapshot.battery.voltage = message->voltage;
        snapshot.battery.current = message->current;
        submit_snapshot(snapshot, "battery");
      });

    rc_subscription_ = create_subscription<rosflight_msgs::msg::RCRaw>(
      "sim/RC", 1,
      [this](rosflight_msgs::msg::RCRaw::ConstSharedPtr message) {
        submit_rc(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "veloxity_sil_board ready: service=sil_board/run, pwm=sim/pwm_output");
  }

private:
  struct FirmwareDeleter
  {
    void operator()(VeloxityFfiHandle * handle) const
    {
      veloxity_sim_destroy(handle);
    }
  };

  bool synchronize_pwm()
  {
    if (!firmware_) {
      return false;
    }

    const auto run_start = std::chrono::steady_clock::now();
    if (last_run_start_) {
      const auto gap = run_start - *last_run_start_;
      if (gap > kWarnSilRunGap) {
        const auto gap_us = std::chrono::duration_cast<std::chrono::microseconds>(gap).count();
        const auto missed_periods =
          gap_us / std::chrono::duration_cast<std::chrono::microseconds>(kExpectedSilRunPeriod).count();
        RCLCPP_WARN(
          get_logger(),
          "sil_board/run service gap: %ld us (~%ld x 400 Hz periods)",
          static_cast<long>(gap_us),
          static_cast<long>(missed_periods));
      }
    }
    last_run_start_ = run_start;

    if (!veloxity_sim_sync_latest_imu(firmware_.get())) {
      RCLCPP_WARN(get_logger(), "timed out waiting for Veloxity firmware IMU processing");
      return false;
    }

    std::array<uint16_t, kPwmChannelCount> outputs{};
    outputs.fill(kDisabledPwmMicros);
    const auto copied = veloxity_sim_get_pwm(firmware_.get(), outputs.data(), outputs.size());
    if (copied != outputs.size()) {
      RCLCPP_WARN(get_logger(), "Veloxity returned %zu PWM channels", copied);
    }
    pwm_outputs_ = outputs;
    publish_pwm();
    const auto duration = std::chrono::steady_clock::now() - run_start;
    if (duration > kWarnSilRunDuration) {
      const auto duration_us =
        std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
      RCLCPP_WARN(
        get_logger(),
        "sil_board/run service duration: %ld us",
        static_cast<long>(duration_us));
    }
    return true;
  }

  bool submit_snapshot(const VeloxityFfiSensorSnapshot & snapshot, const char * sensor_name)
  {
    if (!firmware_ || !veloxity_sim_set_sensors(firmware_.get(), &snapshot)) {
      RCLCPP_WARN(get_logger(), "failed to submit %s sample to Veloxity", sensor_name);
      return false;
    }
    return true;
  }

  void submit_rc(const rosflight_msgs::msg::RCRaw & message)
  {
    VeloxityFfiSensorSnapshot snapshot{};
    snapshot.has_rc = true;
    snapshot.rc.timestamp_us = fcu_clock_micros();
    for (std::size_t i = 0; i < message.values.size(); ++i) {
      snapshot.rc.values[i] = message.values[i];
    }
    submit_snapshot(snapshot, "RC");
  }

  void initialize_default_rc()
  {
    latest_rc_.values.fill(kRcCenterMicros);
    latest_rc_.values[2] = kRcLowMicros;  // throttle/F
    latest_rc_.values[4] = kRcLowMicros;  // C SIL cached default before first RC message
    latest_rc_.values[5] = kRcLowMicros;  // C SIL cached default before first RC message
  }

  uint64_t fcu_clock_micros() const
  {
    return firmware_ ? veloxity_sim_clock_micros(firmware_.get()) : 0;
  }

  void publish_pwm()
  {
    rosflight_msgs::msg::PwmOutput message;
    message.header.stamp = now();
    message.values = pwm_outputs_;
    pwm_publisher_->publish(message);
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_service_;
  rclcpp::Publisher<rosflight_msgs::msg::PwmOutput>::SharedPtr pwm_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
  rclcpp::Subscription<rosflight_msgs::msg::Barometer>::SharedPtr baro_subscription_;
  rclcpp::Subscription<rosflight_msgs::msg::GNSS>::SharedPtr gnss_subscription_;
  rclcpp::Subscription<rosflight_msgs::msg::Airspeed>::SharedPtr diff_pressure_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_subscription_;
  rclcpp::Subscription<rosflight_msgs::msg::BatteryStatus>::SharedPtr battery_subscription_;
  rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_subscription_;

  sensor_msgs::msg::Temperature latest_imu_temperature_;
  rosflight_msgs::msg::RCRaw latest_rc_;

  bool imu_temperature_available_{false};

  std::array<uint16_t, kPwmChannelCount> pwm_outputs_{};
  std::unique_ptr<VeloxityFfiHandle, FirmwareDeleter> firmware_;
  std::optional<std::chrono::steady_clock::time_point> last_run_start_;
};
}  // namespace veloxity_sil_board_shim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<veloxity_sil_board_shim::VeloxitySilBoard>());
  rclcpp::shutdown();
  return 0;
}
