#!/usr/bin/env python3
"""Temporary quad-X angle-mode offboard experiment.

Publishes rosflight_msgs/Command directly in firmware
MODE_ROLL_PITCH_YAWRATE_THROTTLE. This intentionally bypasses ROScopter's
force/torque-to-mixer passthrough path.
"""

import math

import rclpy
from rclpy.node import Node

from rosflight_msgs.msg import Command, SimState


def clamp(value, low, high):
    return max(low, min(high, value))


class QuadXAngleHold(Node):
    def __init__(self):
        super().__init__("quadx_angle_hold_experiment")
        self.command_pub = self.create_publisher(Command, "/command", 10)
        self.truth_sub = self.create_subscription(
            SimState, "/sim/truth_state", self.truth_callback, 10
        )
        self.truth = None
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / 100.0, self.publish_command)
        self.log_timer = self.create_timer(0.5, self.log_state)

        self.target_n = 0.0
        self.target_e = 0.0
        self.target_alt = 5.0
        self.target_yaw = 0.0

        self.get_logger().info(
            "Publishing /command in MODE_ROLL_PITCH_YAWRATE_THROTTLE "
            "for quad-X angle-mode hold"
        )

    def truth_callback(self, msg):
        self.truth = msg

    def publish_command(self):
        msg = Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
        msg.ignore = Command.IGNORE_NONE

        if self.truth is None:
            msg.u[2] = 0.0
            self.command_pub.publish(msg)
            return

        p = self.truth.pose.position
        v = self.truth.twist.linear
        q = self.truth.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        alt = -p.z
        alt_rate = -v.z

        n_err = self.target_n - p.x
        e_err = self.target_e - p.y
        alt_err = self.target_alt - alt

        # Position outer loop to acceleration, then small-angle mapping:
        # NED a_n ~= -g * theta, a_e ~= g * phi.
        a_n = clamp(0.45 * n_err - 0.7 * v.x, -2.0, 2.0)
        a_e = clamp(0.45 * e_err - 0.7 * v.y, -2.0, 2.0)
        phi = clamp(a_e / 9.80665, -0.25, 0.25)
        theta = clamp(-a_n / 9.80665, -0.25, 0.25)

        throttle = clamp(0.62 + 0.055 * alt_err - 0.045 * alt_rate, 0.42, 0.78)
        yaw_err = math.atan2(
            math.sin(self.target_yaw - yaw), math.cos(self.target_yaw - yaw)
        )
        yaw_rate = clamp(1.0 * yaw_err, -0.6, 0.6)

        msg.u[0] = 0.0
        msg.u[1] = 0.0
        msg.u[2] = float(throttle)
        msg.u[3] = float(phi)
        msg.u[4] = float(theta)
        msg.u[5] = float(yaw_rate)
        self.command_pub.publish(msg)

    def log_state(self):
        if self.truth is None:
            self.get_logger().info("Waiting for /sim/truth_state")
            return

        p = self.truth.pose.position
        v = self.truth.twist.linear
        self.get_logger().info(
            "truth n={:.2f} e={:.2f} alt={:.2f} vn={:.2f} ve={:.2f} vz={:.2f}".format(
                p.x, p.y, -p.z, v.x, v.y, v.z
            )
        )


def main():
    rclpy.init()
    node = QuadXAngleHold()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
