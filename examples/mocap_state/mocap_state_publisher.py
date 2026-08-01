#!/usr/bin/env python3
"""Publish a mocap-authoritative, inertial-aided ROScopter State.

Mocap owns position and orientation. Firmware IMU acceleration predicts
velocity between mocap corrections; it never displaces the mocap position
reference. Firmware attitude telemetry supplies body rates and relative
attitude motion during a tightly bounded mocap gap.
"""

import math
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from roscopter_msgs.msg import State
from rosflight_msgs.msg import Attitude
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from mocap_state_math import (
    AlphaBetaFilter3D,
    add,
    norm,
    quaternion_angle,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_normalize,
    quaternion_rotate,
    quaternion_to_euler,
    scale,
    subtract,
)


POSE_STAMPED = "geometry_msgs/msg/PoseStamped"
TRANSFORM_STAMPED = "geometry_msgs/msg/TransformStamped"


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def finite(values):
    return all(math.isfinite(value) for value in values)


class MocapStatePublisher(Node):
    def __init__(self):
        super().__init__("mocap_state_publisher")

        self.mocap_topic = str(
            self.declare_parameter("mocap_topic", "/mocap/vehicle/pose").value
        )
        self.mocap_message_type = str(
            self.declare_parameter("mocap_message_type", POSE_STAMPED).value
        )
        self.mocap_qos_depth = int(
            self.declare_parameter("mocap_qos_depth", 50).value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/estimated_state").value
        )
        self.firmware_attitude_topic = str(
            self.declare_parameter("firmware_attitude_topic", "/attitude").value
        )
        self.firmware_imu_topic = str(
            self.declare_parameter("firmware_imu_topic", "/imu/data").value
        )
        self.frame_id = str(self.declare_parameter("frame_id", "NED").value)

        self.alpha = float(self.declare_parameter("alpha", 0.25).value)
        self.beta = float(self.declare_parameter("beta", 0.01).value)
        self.minimum_valid_samples = int(
            self.declare_parameter("minimum_valid_samples", 30).value
        )
        self.max_mocap_age_s = (
            float(self.declare_parameter("max_mocap_age_ms", 15.0).value) * 1.0e-3
        )
        self.max_filter_gap_s = (
            float(self.declare_parameter("max_filter_gap_ms", 50.0).value) * 1.0e-3
        )
        self.minimum_filter_dt_s = (
            float(self.declare_parameter("minimum_filter_dt_ms", 1.0).value)
            * 1.0e-3
        )
        self.max_position_innovation = float(
            self.declare_parameter("max_position_innovation_m", 0.25).value
        )
        self.max_orientation_jump = float(
            self.declare_parameter("max_orientation_jump_rad", 0.35).value
        )
        self.identical_pose_position_epsilon = float(
            self.declare_parameter("identical_pose_position_epsilon_m", 1.0e-7).value
        )
        self.identical_pose_orientation_epsilon = float(
            self.declare_parameter(
                "identical_pose_orientation_epsilon_rad", 1.0e-7
            ).value
        )
        self.max_identical_pose_age_s = (
            float(self.declare_parameter("max_identical_pose_age_ms", 100.0).value)
            * 1.0e-3
        )

        self.use_firmware_body_rates = bool(
            self.declare_parameter("use_firmware_body_rates", True).value
        )
        self.max_firmware_attitude_age_s = (
            float(
                self.declare_parameter("max_firmware_attitude_age_ms", 20.0).value
            )
            * 1.0e-3
        )
        self.use_firmware_imu = bool(
            self.declare_parameter("use_firmware_imu", True).value
        )
        self.max_firmware_imu_age_s = (
            float(self.declare_parameter("max_firmware_imu_age_ms", 20.0).value)
            * 1.0e-3
        )
        self.gravity_mps2 = float(
            self.declare_parameter("gravity_mps2", 9.80665).value
        )
        self.accelerometer_bias_body = self.vector_parameter(
            "accelerometer_bias_body_mps2", [0.0, 0.0, 0.0]
        )
        self.imu_accel_lpf_alpha = float(
            self.declare_parameter("imu_accel_lpf_alpha", 0.20).value
        )
        self.bridge_with_firmware_attitude = bool(
            self.declare_parameter("bridge_with_firmware_attitude", False).value
        )
        self.max_bridge_age_s = (
            float(self.declare_parameter("max_bridge_age_ms", 30.0).value) * 1.0e-3
        )
        self.bridge_publish_hz = float(
            self.declare_parameter("bridge_publish_hz", 360.0).value
        )
        self.allow_zero_timestamp = bool(
            self.declare_parameter("allow_zero_timestamp", False).value
        )

        self.room_to_ned_q = self.quaternion_parameter(
            "room_to_ned_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0]
        )
        self.body_to_marker_q = self.quaternion_parameter(
            "body_to_marker_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0]
        )
        self.body_origin_in_marker = self.vector_parameter(
            "body_origin_in_marker_m", [0.0, 0.0, 0.0]
        )
        self.origin_at_first_sample = bool(
            self.declare_parameter("origin_at_first_sample", True).value
        )
        self.configured_origin = self.vector_parameter(
            "ned_origin_in_room_m", [0.0, 0.0, 0.0]
        )
        self.initial_lat = float(self.declare_parameter("initial_lat", 0.0).value)
        self.initial_lon = float(self.declare_parameter("initial_lon", 0.0).value)
        self.initial_alt = float(self.declare_parameter("initial_alt", 0.0).value)

        self.validate_parameters()
        self.filter = AlphaBetaFilter3D(self.alpha, self.beta)
        self.origin_ned = None if self.origin_at_first_sample else tuple(
            quaternion_rotate(self.room_to_ned_q, self.configured_origin)
        )
        self.last_source_time = None
        self.last_mocap_monotonic = None
        self.last_mocap_q = None
        self.last_observed_position = None
        self.last_observed_q = None
        self.last_pose_change_monotonic = None
        self.pose_frozen = False
        self.valid_sample_count = 0
        self.tracking_valid = False
        self.last_health_published = None
        self.conflict = False
        self.firmware_q = None
        self.firmware_rates = None
        self.firmware_monotonic = None
        self.firmware_alignment = None
        self.firmware_imu_alignment = None
        self.firmware_imu_q = None
        self.firmware_acceleration_body = None
        self.firmware_imu_monotonic = None
        self.acceleration_ned = None
        self.last_log_times = {}

        self.state_publisher = self.create_publisher(State, self.output_topic, 10)
        health_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.health_publisher = self.create_publisher(
            Bool, "~/tracking_valid", health_qos
        )
        self.attitude_subscription = self.create_subscription(
            Attitude,
            self.firmware_attitude_topic,
            self.attitude_callback,
            qos_profile_sensor_data,
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            self.firmware_imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        if self.mocap_message_type == POSE_STAMPED:
            message_class = PoseStamped
        elif self.mocap_message_type == TRANSFORM_STAMPED:
            message_class = TransformStamped
        else:
            raise ValueError(
                f"mocap_message_type must be {POSE_STAMPED} or {TRANSFORM_STAMPED}"
            )
        mocap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.mocap_qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.mocap_subscription = self.create_subscription(
            message_class,
            self.mocap_topic,
            self.mocap_callback,
            mocap_qos,
        )

        self.health_timer = self.create_timer(
            1.0 / self.bridge_publish_hz, self.health_and_bridge_callback
        )
        self.conflict_timer = self.create_timer(0.5, self.conflict_callback)
        self.publish_health(False)
        self.get_logger().info(
            "Mocap owns %s from %s (%s); alpha=%.3f beta=%.3f, "
            "firmware IMU aiding=%s, body rates=%s, short-gap bridge=%s"
            % (
                self.output_topic,
                self.mocap_topic,
                self.mocap_message_type,
                self.alpha,
                self.beta,
                self.use_firmware_imu,
                self.use_firmware_body_rates,
                self.bridge_with_firmware_attitude,
            )
        )

    def quaternion_parameter(self, name, default):
        values = tuple(float(x) for x in self.declare_parameter(name, default).value)
        if len(values) != 4:
            raise ValueError(f"{name} must contain four xyzw values")
        return quaternion_normalize(values)

    def vector_parameter(self, name, default):
        values = tuple(float(x) for x in self.declare_parameter(name, default).value)
        if len(values) != 3 or not finite(values):
            raise ValueError(f"{name} must contain three finite values")
        return values

    def validate_parameters(self):
        if self.mocap_qos_depth < 1:
            raise ValueError("mocap_qos_depth must be positive")
        if self.minimum_valid_samples < 1:
            raise ValueError("minimum_valid_samples must be positive")
        if self.max_mocap_age_s <= 0.0:
            raise ValueError("max_mocap_age_ms must be positive")
        if self.max_filter_gap_s <= self.max_mocap_age_s:
            raise ValueError("max_filter_gap_ms must exceed max_mocap_age_ms")
        if not 0.0 < self.minimum_filter_dt_s < self.max_filter_gap_s:
            raise ValueError(
                "minimum_filter_dt_ms must be positive and below max_filter_gap_ms"
            )
        if self.max_position_innovation <= 0.0:
            raise ValueError("max_position_innovation_m must be positive")
        if not 0.0 < self.max_orientation_jump <= math.pi:
            raise ValueError("max_orientation_jump_rad must be in (0, pi]")
        if self.identical_pose_position_epsilon < 0.0:
            raise ValueError("identical_pose_position_epsilon_m must be non-negative")
        if not 0.0 <= self.identical_pose_orientation_epsilon <= math.pi:
            raise ValueError(
                "identical_pose_orientation_epsilon_rad must be in [0, pi]"
            )
        if self.max_identical_pose_age_s <= 0.0:
            raise ValueError("max_identical_pose_age_ms must be positive")
        if self.max_firmware_attitude_age_s <= 0.0:
            raise ValueError("max_firmware_attitude_age_ms must be positive")
        if self.max_firmware_imu_age_s <= 0.0:
            raise ValueError("max_firmware_imu_age_ms must be positive")
        if not math.isfinite(self.gravity_mps2) or self.gravity_mps2 <= 0.0:
            raise ValueError("gravity_mps2 must be positive and finite")
        if not 0.0 < self.imu_accel_lpf_alpha <= 1.0:
            raise ValueError("imu_accel_lpf_alpha must be in (0, 1]")
        if self.bridge_publish_hz <= 0.0:
            raise ValueError("bridge_publish_hz must be positive")
        if self.bridge_with_firmware_attitude:
            if self.max_bridge_age_s <= self.max_mocap_age_s:
                raise ValueError(
                    "max_bridge_age_ms must exceed max_mocap_age_ms when bridging"
                )

    def throttled_log(self, level, key, message, interval=1.0):
        now = time.monotonic()
        if now - self.last_log_times.get(key, -math.inf) < interval:
            return
        self.last_log_times[key] = now
        getattr(self.get_logger(), level)(message)

    def publish_health(self, valid):
        self.tracking_valid = valid
        if valid == self.last_health_published:
            return
        message = Bool()
        message.data = valid
        self.health_publisher.publish(message)
        self.last_health_published = valid

    def invalidate(self, reason):
        if self.tracking_valid:
            self.get_logger().error(reason + "; stopping estimated-state publication")
        else:
            self.throttled_log("warn", reason, reason)
        self.publish_health(False)

    def attitude_callback(self, message):
        q = (
            message.attitude.x,
            message.attitude.y,
            message.attitude.z,
            message.attitude.w,
        )
        rates = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        if not finite(q + rates):
            self.throttled_log(
                "warn", "bad firmware attitude", "Ignoring non-finite firmware attitude"
            )
            return
        try:
            self.firmware_q = quaternion_normalize(q)
        except ValueError:
            self.throttled_log(
                "warn", "bad firmware quaternion", "Ignoring invalid firmware quaternion"
            )
            return
        self.firmware_rates = rates
        self.firmware_monotonic = time.monotonic()

    def imu_callback(self, message):
        q = (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        )
        acceleration = (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        if not finite(q + acceleration):
            self.throttled_log(
                "warn", "bad firmware imu", "Ignoring non-finite firmware IMU"
            )
            return
        try:
            self.firmware_imu_q = quaternion_normalize(q)
        except ValueError:
            self.throttled_log(
                "warn",
                "bad firmware imu quaternion",
                "Ignoring invalid firmware IMU quaternion",
            )
            return
        self.firmware_acceleration_body = acceleration
        self.firmware_imu_monotonic = time.monotonic()
        self.refresh_acceleration_ned()

    def refresh_acceleration_ned(self):
        if (
            self.firmware_imu_alignment is None
            or self.firmware_imu_q is None
            or self.firmware_acceleration_body is None
        ):
            return
        aligned_body_q_ned = quaternion_normalize(
            quaternion_multiply(self.firmware_imu_alignment, self.firmware_imu_q)
        )
        specific_force_body = subtract(
            self.firmware_acceleration_body, self.accelerometer_bias_body
        )
        acceleration_ned = add(
            quaternion_rotate(aligned_body_q_ned, specific_force_body),
            (0.0, 0.0, self.gravity_mps2),
        )
        if self.acceleration_ned is None:
            self.acceleration_ned = acceleration_ned
        else:
            alpha = self.imu_accel_lpf_alpha
            self.acceleration_ned = add(
                scale(self.acceleration_ned, 1.0 - alpha),
                scale(acceleration_ned, alpha),
            )

    def extract_mocap(self, message):
        if isinstance(message, PoseStamped):
            translation = message.pose.position
            rotation = message.pose.orientation
        else:
            translation = message.transform.translation
            rotation = message.transform.rotation
        return (
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            stamp_seconds(message.header.stamp),
        )

    def transform_mocap_pose(self, room_position, room_marker_q):
        room_marker_q = quaternion_normalize(room_marker_q)
        body_position_room = add(
            room_position,
            quaternion_rotate(room_marker_q, self.body_origin_in_marker),
        )
        body_position_ned_absolute = quaternion_rotate(
            self.room_to_ned_q, body_position_room
        )
        body_q_ned = quaternion_normalize(
            quaternion_multiply(
                quaternion_multiply(self.room_to_ned_q, room_marker_q),
                self.body_to_marker_q,
            )
        )
        if self.origin_ned is None:
            self.origin_ned = body_position_ned_absolute
            self.get_logger().info(
                "Established mocap NED position origin at [%.4f, %.4f, %.4f]"
                % self.origin_ned
            )
        return subtract(body_position_ned_absolute, self.origin_ned), body_q_ned

    def reset_tracking(self, position, source_time, mocap_q, now_monotonic, reason):
        self.filter.reset(position)
        self.last_source_time = source_time
        self.last_mocap_monotonic = now_monotonic
        self.last_mocap_q = mocap_q
        self.valid_sample_count = 1
        self.firmware_alignment = None
        self.firmware_imu_alignment = None
        self.acceleration_ned = None
        self.update_firmware_alignment(mocap_q, now_monotonic)
        self.publish_health(False)
        self.throttled_log("warn", "tracking reset", reason)

    def update_firmware_alignment(self, mocap_q, now_monotonic):
        if self.imu_is_fresh(now_monotonic):
            self.firmware_imu_alignment = quaternion_normalize(
                quaternion_multiply(
                    mocap_q, quaternion_conjugate(self.firmware_imu_q)
                )
            )
            self.refresh_acceleration_ned()

    def check_frozen_pose(self, position, mocap_q, now_monotonic):
        if self.last_observed_position is None:
            self.last_observed_position = position
            self.last_observed_q = mocap_q
            self.last_pose_change_monotonic = now_monotonic
            return False, False

        changed = (
            norm(subtract(position, self.last_observed_position))
            > self.identical_pose_position_epsilon
            or quaternion_angle(mocap_q, self.last_observed_q)
            > self.identical_pose_orientation_epsilon
        )
        if changed:
            recovered = self.pose_frozen
            self.last_observed_position = position
            self.last_observed_q = mocap_q
            self.last_pose_change_monotonic = now_monotonic
            self.pose_frozen = False
            return False, recovered

        identical_age = now_monotonic - self.last_pose_change_monotonic
        if identical_age > self.max_identical_pose_age_s:
            self.pose_frozen = True
            self.invalidate(
                "Mocap pose is bit-for-bit unchanged for %.1f ms"
                % (identical_age * 1000.0)
            )
            return True, False
        return False, False

    def mocap_callback(self, message):
        if self.conflict:
            return
        now_monotonic = time.monotonic()
        room_position, room_marker_q, source_time = self.extract_mocap(message)
        if not finite(room_position + room_marker_q):
            self.invalidate("Mocap pose contains non-finite values")
            return
        if source_time == 0.0:
            if not self.allow_zero_timestamp:
                self.invalidate("Mocap message has a zero timestamp")
                return
            source_time = self.get_clock().now().nanoseconds * 1.0e-9

        try:
            position, mocap_q = self.transform_mocap_pose(
                room_position, room_marker_q
            )
        except ValueError as error:
            self.invalidate(f"Invalid mocap quaternion: {error}")
            return

        frozen, recovered_from_frozen = self.check_frozen_pose(
            position, mocap_q, now_monotonic
        )
        if frozen:
            return
        if recovered_from_frozen:
            self.reset_tracking(
                position,
                source_time,
                mocap_q,
                now_monotonic,
                "Mocap pose changed after a frozen-pose fault; resetting tracking",
            )
            return

        if not self.filter.initialized:
            self.reset_tracking(
                position, source_time, mocap_q, now_monotonic, "Initializing mocap tracking"
            )
            return

        dt = source_time - self.last_source_time
        if not math.isfinite(dt) or dt <= 0.0:
            self.invalidate("Mocap timestamps are non-monotonic")
            return
        if dt < self.minimum_filter_dt_s:
            # Some VRPN sources deliver several samples only microseconds apart
            # and then pause for the remainder of their physical update period.
            # Feeding those tiny intervals to beta/dt creates enormous false
            # velocities from micrometer-scale position noise. Treat them as
            # one batch while still recording that mocap transport is fresh.
            self.last_mocap_monotonic = now_monotonic
            return
        if dt > self.max_filter_gap_s:
            self.reset_tracking(
                position,
                source_time,
                mocap_q,
                now_monotonic,
                f"Mocap gap {dt * 1000.0:.1f} ms reset the tracking filter",
            )
            return

        acceleration = (0.0, 0.0, 0.0)
        if self.use_firmware_imu:
            if not self.imu_is_fresh(now_monotonic) or self.acceleration_ned is None:
                self.invalidate("Firmware IMU is stale; inertial aiding unavailable")
                return
            acceleration = self.acceleration_ned

        innovation = self.filter.innovation(position, dt, acceleration)
        if norm(innovation) > self.max_position_innovation:
            self.invalidate(
                "Mocap position innovation %.3f m exceeds %.3f m"
                % (norm(innovation), self.max_position_innovation)
            )
            return
        angle_jump = quaternion_angle(self.last_mocap_q, mocap_q)
        if angle_jump > self.max_orientation_jump:
            self.invalidate(
                "Mocap orientation jump %.3f rad exceeds %.3f rad"
                % (angle_jump, self.max_orientation_jump)
            )
            return

        self.filter.update(position, dt, acceleration)
        self.last_source_time = source_time
        self.last_mocap_monotonic = now_monotonic
        self.last_mocap_q = mocap_q
        self.valid_sample_count += 1

        if self.firmware_is_fresh(now_monotonic):
            self.firmware_alignment = quaternion_normalize(
                quaternion_multiply(mocap_q, quaternion_conjugate(self.firmware_q))
            )
        self.update_firmware_alignment(mocap_q, now_monotonic)

        if self.valid_sample_count < self.minimum_valid_samples:
            self.publish_health(False)
            return

        self.publish_state(self.filter.position, self.filter.velocity, mocap_q)
        self.publish_health(True)

    def firmware_is_fresh(self, now_monotonic):
        return (
            self.firmware_q is not None
            and self.firmware_rates is not None
            and self.firmware_monotonic is not None
            and now_monotonic - self.firmware_monotonic
            <= self.max_firmware_attitude_age_s
        )

    def imu_is_fresh(self, now_monotonic):
        return (
            self.firmware_imu_q is not None
            and self.firmware_acceleration_body is not None
            and self.firmware_imu_monotonic is not None
            and now_monotonic - self.firmware_imu_monotonic
            <= self.max_firmware_imu_age_s
        )

    def publish_state(self, position_ned, velocity_ned, attitude_q):
        now_monotonic = time.monotonic()
        body_velocity = quaternion_rotate(
            quaternion_conjugate(attitude_q), velocity_ned
        )
        phi, theta, psi = quaternion_to_euler(attitude_q)
        rates = (0.0, 0.0, 0.0)
        if self.use_firmware_body_rates:
            if self.firmware_is_fresh(now_monotonic):
                rates = self.firmware_rates
            else:
                self.throttled_log(
                    "warn",
                    "stale firmware rates",
                    "Firmware attitude is stale; publishing zero body rates",
                )

        message = State()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.p_n, message.p_e, message.p_d = position_ned
        message.v_x, message.v_y, message.v_z = body_velocity
        message.p, message.q, message.r = rates
        message.phi = phi
        message.theta = theta
        message.psi = psi
        message.b_x = 0.0
        message.b_y = 0.0
        message.b_z = 0.0
        message.quat.x, message.quat.y, message.quat.z, message.quat.w = attitude_q
        message.initial_lat = self.initial_lat
        message.initial_lon = self.initial_lon
        message.initial_alt = self.initial_alt
        self.state_publisher.publish(message)

    def health_and_bridge_callback(self):
        if self.conflict or self.last_mocap_monotonic is None:
            return
        now_monotonic = time.monotonic()
        age = now_monotonic - self.last_mocap_monotonic
        if age <= self.max_mocap_age_s:
            return

        can_bridge = (
            self.bridge_with_firmware_attitude
            and not self.pose_frozen
            and age <= self.max_bridge_age_s
            and self.firmware_alignment is not None
            and self.firmware_is_fresh(now_monotonic)
            and (
                not self.use_firmware_imu
                or (
                    self.imu_is_fresh(now_monotonic)
                    and self.acceleration_ned is not None
                )
            )
            and self.valid_sample_count >= self.minimum_valid_samples
        )
        if can_bridge:
            attitude_q = quaternion_normalize(
                quaternion_multiply(self.firmware_alignment, self.firmware_q)
            )
            acceleration = (
                self.acceleration_ned
                if self.use_firmware_imu
                else (0.0, 0.0, 0.0)
            )
            position, velocity = self.filter.predicted_state(age, acceleration)
            self.publish_state(position, velocity, attitude_q)
            self.publish_health(True)
            self.throttled_log(
                "warn",
                "bridging mocap",
                "Bridging a short mocap gap with firmware attitude and acceleration",
            )
            return

        self.invalidate(f"Mocap is stale by {age * 1000.0:.1f} ms")

    def conflict_callback(self):
        if self.conflict:
            return
        resolved_topic = self.resolve_topic_name(self.output_topic)
        publishers = self.get_publishers_info_by_topic(resolved_topic)
        if len(publishers) <= 1:
            return
        owners = ", ".join(
            sorted(f"{info.node_namespace}/{info.node_name}" for info in publishers)
        )
        self.conflict = True
        self.publish_health(False)
        self.get_logger().fatal(
            f"Multiple publishers own {resolved_topic}: {owners}. "
            "Run either v_start_estimator or v_start_mocap, never both."
        )


def main():
    rclpy.init()
    node = None
    try:
        node = MocapStatePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
