#!/usr/bin/env python3
"""Add velocity-reference feed-forward for the upstream trajectory follower.

The upstream follower implements ``kp * position_error - kd * velocity`` and
does not consume TrajectoryCommand.velocity.  Advancing the position reference
by ``kd / kp * commanded_velocity`` makes that expression equivalent to the
usual ``kp * position_error + kd * velocity_error`` without changing upstream.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError

from roscopter_msgs.msg import TrajectoryCommand


class TrajectoryVelocityAdapter(Node):
    def __init__(self):
        super().__init__("trajectory_velocity_adapter")
        north_kp = self.declare_parameter("north_kp", 1.5).value
        north_kd = self.declare_parameter("north_kd", 3.5).value
        east_kp = self.declare_parameter("east_kp", 1.5).value
        east_kd = self.declare_parameter("east_kd", 3.5).value
        down_kp = self.declare_parameter("down_kp", 4.0).value
        down_kd = self.declare_parameter("down_kd", 3.5).value
        if north_kp <= 0.0 or east_kp <= 0.0 or down_kp <= 0.0:
            raise ValueError("Position gains must be positive")
        self.north_lead = north_kd / north_kp
        self.east_lead = east_kd / east_kp
        self.down_lead = down_kd / down_kp
        self.publisher = self.create_publisher(
            TrajectoryCommand, "/trajectory_command_compensated", 10
        )
        self.subscription = self.create_subscription(
            TrajectoryCommand, "/trajectory_command", self.adapt, 10
        )
        self.get_logger().info(
            "Adding trajectory velocity feed-forward with north/east/down lead "
            f"{self.north_lead:.3f}/{self.east_lead:.3f}/"
            f"{self.down_lead:.3f} s"
        )

    def adapt(self, source):
        # Path manager emits a NaN placeholder before receiving its first
        # waypoint. It is already a valid deserialized ROS message, but the
        # generated Python setters reject reconstructing it. Pass it through;
        # the follower marks the resulting command invalid during RC override.
        if not all(
            map(
                math.isfinite,
                (*source.position, *source.velocity, *source.acceleration),
            )
        ):
            self.publisher.publish(source)
            return
        source.position[0] += self.north_lead * source.velocity[0]
        source.position[1] += self.east_lead * source.velocity[1]
        source.position[2] += self.down_lead * source.velocity[2]
        self.publisher.publish(source)


def main():
    rclpy.init()
    node = TrajectoryVelocityAdapter()
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
