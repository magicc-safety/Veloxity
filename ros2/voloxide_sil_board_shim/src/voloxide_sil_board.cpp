#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
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

#include "voloxide_sil_board_shim/voloxide_ffi.h"

namespace voloxide_sil_board_shim
{
namespace
{
constexpr std::size_t kPwmChannelCount = 14;
constexpr uint16_t kDisabledPwmMicros = 1000;

uint64_t stamp_to_micros(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint64_t>(stamp.sec) * 1000000 + stamp.nanosec / 1000;
}

VoloxideFfiVector3 vector_to_ffi(const geometry_msgs::msg::Vector3 & vector)
{
  return VoloxideFfiVector3{vector.x, vector.y, vector.z};
}
}

class VoloxideSilBoard final : public rclcpp::Node
{
public:
  VoloxideSilBoard()
  : rclcpp::Node("voloxide_sil_board")
  {
    declare_parameter<std::string>("simulation_host", "localhost");
    declare_parameter<int>("simulation_port", 14525);
    declare_parameter<std::string>("ROS_host", "localhost");
    declare_parameter<int>("ROS_port", 14520);
    declare_parameter<int64_t>("serial_delay_ns", 6000000);

    pwm_outputs_.fill(kDisabledPwmMicros);
    firmware_.reset(voloxide_sim_create());
    if (!firmware_) {
      RCLCPP_ERROR(
        get_logger(),
        "failed to initialize Voloxide FFI; check MAVLink UDP port availability");
    }

    run_service_ = create_service<std_srvs::srv::Trigger>(
      "sil_board/run",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        response->success = run_once();
        response->message = response->success ? "Voloxide SIL iteration completed" :
          "Voloxide SIL iteration failed";
      });

    pwm_publisher_ = create_publisher<rosflight_msgs::msg::PwmOutput>("sim/pwm_output", 1);

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "sim/sensors/imu/data", 1,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        latest_imu_ = *message;
        imu_available_ = true;
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
        latest_mag_ = *message;
        mag_available_ = true;
      });

    baro_subscription_ = create_subscription<rosflight_msgs::msg::Barometer>(
      "sim/sensors/baro", 1,
      [this](rosflight_msgs::msg::Barometer::ConstSharedPtr message) {
        latest_baro_ = *message;
        baro_available_ = true;
      });

    gnss_subscription_ = create_subscription<rosflight_msgs::msg::GNSS>(
      "sim/sensors/gnss", 1,
      [this](rosflight_msgs::msg::GNSS::ConstSharedPtr message) {
        latest_gnss_ = *message;
        gnss_available_ = true;
      });

    diff_pressure_subscription_ = create_subscription<rosflight_msgs::msg::Airspeed>(
      "sim/sensors/diff_pressure", 1,
      [this](rosflight_msgs::msg::Airspeed::ConstSharedPtr message) {
        latest_diff_pressure_ = *message;
        diff_pressure_available_ = true;
      });

    range_subscription_ = create_subscription<sensor_msgs::msg::Range>(
      "sim/sensors/range", 1,
      [this](sensor_msgs::msg::Range::ConstSharedPtr message) {
        latest_range_ = *message;
        range_available_ = true;
      });

    battery_subscription_ = create_subscription<rosflight_msgs::msg::BatteryStatus>(
      "sim/sensors/battery", 1,
      [this](rosflight_msgs::msg::BatteryStatus::ConstSharedPtr message) {
        latest_battery_ = *message;
        battery_available_ = true;
      });

    rc_subscription_ = create_subscription<rosflight_msgs::msg::RCRaw>(
      "sim/RC", 1,
      [this](rosflight_msgs::msg::RCRaw::ConstSharedPtr message) {
        latest_rc_ = *message;
        rc_available_ = true;
      });

    RCLCPP_INFO(
      get_logger(),
      "voloxide_sil_board ready: service=sil_board/run, pwm=sim/pwm_output");
  }

private:
  struct FirmwareDeleter
  {
    void operator()(VoloxideFfiHandle * handle) const
    {
      voloxide_sim_destroy(handle);
    }
  };

  bool run_once()
  {
    if (!firmware_) {
      return false;
    }

    auto snapshot = build_sensor_snapshot();
    if (!voloxide_sim_set_sensors(firmware_.get(), &snapshot)) {
      RCLCPP_WARN(get_logger(), "failed to pass sensor snapshot to Voloxide");
      return false;
    }

    for (int iteration = 0; iteration < 2; ++iteration) {
      if (!voloxide_sim_run_once(firmware_.get())) {
        RCLCPP_WARN(get_logger(), "Voloxide firmware iteration %d failed", iteration + 1);
        return false;
      }
    }

    std::array<uint16_t, kPwmChannelCount> outputs{};
    outputs.fill(kDisabledPwmMicros);
    const auto copied = voloxide_sim_get_pwm(firmware_.get(), outputs.data(), outputs.size());
    if (copied != outputs.size()) {
      RCLCPP_WARN(get_logger(), "Voloxide returned %zu PWM channels", copied);
    }
    pwm_outputs_ = outputs;
    publish_pwm();
    return true;
  }

  VoloxideFfiSensorSnapshot build_sensor_snapshot() const
  {
    VoloxideFfiSensorSnapshot snapshot{};
    const auto timestamp_us = fcu_clock_micros();

    snapshot.has_imu = imu_available_;
    if (snapshot.has_imu) {
      snapshot.imu.timestamp_us = timestamp_us;
      snapshot.imu.angular_velocity = vector_to_ffi(latest_imu_.angular_velocity);
      snapshot.imu.linear_acceleration = vector_to_ffi(latest_imu_.linear_acceleration);
      snapshot.imu.temperature_kelvin = imu_temperature_available_ ?
        static_cast<float>(latest_imu_temperature_.temperature) :
        298.15f;
    }

    snapshot.has_mag = mag_available_;
    if (snapshot.has_mag) {
      snapshot.mag.timestamp_us = timestamp_us;
      snapshot.mag.magnetic_field = vector_to_ffi(latest_mag_.magnetic_field);
    }

    snapshot.has_baro = baro_available_;
    if (snapshot.has_baro) {
      snapshot.baro.timestamp_us = timestamp_us;
      snapshot.baro.altitude = latest_baro_.altitude;
      snapshot.baro.pressure = latest_baro_.pressure;
      snapshot.baro.temperature_kelvin = latest_baro_.temperature;
    }

    snapshot.has_gnss = gnss_available_;
    if (snapshot.has_gnss) {
      snapshot.gnss.timestamp_us = timestamp_us;
      snapshot.gnss.fix_type = latest_gnss_.fix_type;
      snapshot.gnss.num_sat = latest_gnss_.num_sat;
      snapshot.gnss.lat_degrees = latest_gnss_.lat;
      snapshot.gnss.lon_degrees = latest_gnss_.lon;
      snapshot.gnss.alt = latest_gnss_.alt;
      snapshot.gnss.horizontal_accuracy = latest_gnss_.horizontal_accuracy;
      snapshot.gnss.vertical_accuracy = latest_gnss_.vertical_accuracy;
      snapshot.gnss.vel_n = latest_gnss_.vel_n;
      snapshot.gnss.vel_e = latest_gnss_.vel_e;
      snapshot.gnss.vel_d = latest_gnss_.vel_d;
      snapshot.gnss.speed_accuracy = latest_gnss_.speed_accuracy;
      snapshot.gnss.unix_seconds = latest_gnss_.gnss_unix_seconds;
      snapshot.gnss.unix_nanos = latest_gnss_.gnss_unix_nanos;
    }

    snapshot.has_airspeed = diff_pressure_available_;
    if (snapshot.has_airspeed) {
      snapshot.airspeed.timestamp_us = timestamp_us;
      snapshot.airspeed.differential_pressure = latest_diff_pressure_.differential_pressure;
      snapshot.airspeed.temperature_kelvin = latest_diff_pressure_.temperature;
      snapshot.airspeed.indicated_airspeed = latest_diff_pressure_.velocity;
    }

    snapshot.has_range = range_available_;
    if (snapshot.has_range) {
      snapshot.range.timestamp_us = timestamp_us;
      snapshot.range.range = latest_range_.range;
      snapshot.range.min_range = latest_range_.min_range;
      snapshot.range.max_range = latest_range_.max_range;
    }

    snapshot.has_battery = battery_available_;
    if (snapshot.has_battery) {
      snapshot.battery.timestamp_us = timestamp_us;
      snapshot.battery.voltage = latest_battery_.voltage;
      snapshot.battery.current = latest_battery_.current;
    }

    snapshot.has_rc = rc_available_;
    if (snapshot.has_rc) {
      snapshot.rc.timestamp_us = timestamp_us;
      for (std::size_t i = 0; i < latest_rc_.values.size(); ++i) {
        snapshot.rc.values[i] = latest_rc_.values[i];
      }
    }

    return snapshot;
  }

  uint64_t fcu_clock_micros() const
  {
    const auto elapsed = std::chrono::steady_clock::now() - boot_time_;
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
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

  sensor_msgs::msg::Imu latest_imu_;
  sensor_msgs::msg::Temperature latest_imu_temperature_;
  sensor_msgs::msg::MagneticField latest_mag_;
  rosflight_msgs::msg::Barometer latest_baro_;
  rosflight_msgs::msg::GNSS latest_gnss_;
  rosflight_msgs::msg::Airspeed latest_diff_pressure_;
  sensor_msgs::msg::Range latest_range_;
  rosflight_msgs::msg::BatteryStatus latest_battery_;
  rosflight_msgs::msg::RCRaw latest_rc_;

  bool imu_available_{false};
  bool imu_temperature_available_{false};
  bool mag_available_{false};
  bool baro_available_{false};
  bool gnss_available_{false};
  bool diff_pressure_available_{false};
  bool range_available_{false};
  bool battery_available_{false};
  bool rc_available_{false};

  std::array<uint16_t, kPwmChannelCount> pwm_outputs_{};
  std::unique_ptr<VoloxideFfiHandle, FirmwareDeleter> firmware_;
  std::chrono::steady_clock::time_point boot_time_{std::chrono::steady_clock::now()};
};
}  // namespace voloxide_sil_board_shim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<voloxide_sil_board_shim::VoloxideSilBoard>());
  rclcpp::shutdown();
  return 0;
}
