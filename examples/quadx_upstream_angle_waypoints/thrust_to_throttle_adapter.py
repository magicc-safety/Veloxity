#!/usr/bin/env python3
"""Adapt ROScopter trajectory-follower thrust commands to firmware angle mode.

The upstream trajectory follower publishes roll, pitch, yaw-rate, and thrust in
newtons using ControllerCommand mode 10.  The upstream controller already knows
how to forward ControllerCommand mode 6 as ROSflight firmware command mode 2.
This node bridges the unit/mode mismatch without modifying ROScopter.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError

from roscopter_msgs.msg import ControllerCommand


class ThrustToThrottleAdapter(Node):
    def __init__(self):
        super().__init__("thrust_to_throttle_adapter")

        self.mass = float(self.declare_parameter("mass", 2.0).value)
        self.gravity = float(self.declare_parameter("gravity", 9.81).value)
        self.equilibrium_throttle = float(
            self.declare_parameter("equilibrium_throttle", 0.5).value
        )
        self.min_throttle = float(self.declare_parameter("min_throttle", 0.4).value)
        self.max_throttle = float(self.declare_parameter("max_throttle", 0.85).value)
        self.max_roll = float(self.declare_parameter("max_roll_rad", 0.30).value)
        self.max_pitch = float(self.declare_parameter("max_pitch_rad", 0.30).value)
        self.max_yaw_rate = float(
            self.declare_parameter("max_yaw_rate_rad_s", 0.70).value
        )
        self.input_topic = str(
            self.declare_parameter(
                "input_topic", "/high_level_command_thrust"
            ).value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/high_level_command").value
        )

        if self.mass <= 0.0:
            raise ValueError("mass must be positive")
        if self.gravity <= 0.0:
            raise ValueError("gravity must be positive")
        if not 0.0 < self.equilibrium_throttle <= 1.0:
            raise ValueError("equilibrium_throttle must be in (0, 1]")
        if not 0.0 <= self.min_throttle < self.max_throttle <= 1.0:
            raise ValueError("throttle limits must satisfy 0 <= min < max <= 1")

        self.hover_thrust = self.mass * self.gravity
        self.publisher = self.create_publisher(
            ControllerCommand, self.output_topic, 1
        )
        self.subscription = self.create_subscription(
            ControllerCommand, self.input_topic, self.command_callback, 1
        )
        self.last_log_ns = 0

        self.get_logger().info(
            "Adapting %s mode 10 thrust to %s mode 6 throttle: "
            "mass=%.3f kg gravity=%.4f m/s^2 equilibrium_throttle=%.4f"
            % (
                self.input_topic,
                self.output_topic,
                self.mass,
                self.gravity,
                self.equilibrium_throttle,
            )
        )

    def command_callback(self, msg):
        output = ControllerCommand()
        output.header = msg.header
        output.phi_c = msg.phi_c
        output.theta_c = msg.theta_c

        expected_mode = (
            ControllerCommand.MODE_ROLL_PITCH_YAWRATE_THRUST_TO_MIXER
        )
        if msg.mode != expected_mode:
            output.cmd_valid = False
            self.publisher.publish(output)
            self.get_logger().error(
                f"Expected ControllerCommand mode {expected_mode}, got {msg.mode}; "
                "publishing an invalid command"
            )
            return

        throttle = msg.cmd4 / self.hover_thrust * self.equilibrium_throttle
        if not math.isfinite(throttle) or msg.cmd4 < 0.0:
            output.cmd_valid = False
            self.publisher.publish(output)
            self.get_logger().error(
                f"Invalid thrust command {msg.cmd4}; publishing an invalid command"
            )
            return

        output.mode = ControllerCommand.MODE_ROLL_PITCH_YAWRATE_THROTTLE
        output.cmd1 = max(-self.max_roll, min(self.max_roll, msg.cmd1))
        output.cmd2 = max(-self.max_pitch, min(self.max_pitch, msg.cmd2))
        output.cmd3 = max(-self.max_yaw_rate, min(self.max_yaw_rate, msg.cmd3))
        output.cmd4 = max(self.min_throttle, min(self.max_throttle, throttle))
        output.cmd_valid = msg.cmd_valid
        self.publisher.publish(output)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= 1_000_000_000:
            self.last_log_ns = now_ns
            self.get_logger().info(
                "thrust=%.3f N -> throttle=%.4f, attitude=(%.3f, %.3f), "
                "yaw_rate=%.3f valid=%s"
                % (
                    msg.cmd4,
                    output.cmd4,
                    output.cmd1,
                    output.cmd2,
                    output.cmd3,
                    msg.cmd_valid,
                )
            )


def main():
    rclpy.init()
    node = ThrustToThrottleAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
