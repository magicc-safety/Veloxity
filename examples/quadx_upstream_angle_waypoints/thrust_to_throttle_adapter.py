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

from roscopter_msgs.msg import ControllerCommand, State
from rosflight_msgs.msg import Status


class ThrustToThrottleAdapter(Node):
    # ROSflight CommandManager::RCOverrideReason masks. The status message does
    # not currently expose named constants for these values.
    OVERRIDE_ATT_SWITCH = 0x001
    OVERRIDE_X = 0x004
    OVERRIDE_Y = 0x008
    OVERRIDE_OFFBOARD_X_INACTIVE = 0x040
    OVERRIDE_OFFBOARD_Y_INACTIVE = 0x080
    ROLL_OVERRIDE_MASK = (
        OVERRIDE_ATT_SWITCH | OVERRIDE_X | OVERRIDE_OFFBOARD_X_INACTIVE
    )
    PITCH_OVERRIDE_MASK = (
        OVERRIDE_ATT_SWITCH | OVERRIDE_Y | OVERRIDE_OFFBOARD_Y_INACTIVE
    )

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
        self.min_projection_cosine = float(
            self.declare_parameter("min_projection_cosine", 0.25).value
        )
        self.input_topic = str(
            self.declare_parameter(
                "input_topic", "/high_level_command_thrust"
            ).value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/high_level_command").value
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/estimated_state").value
        )
        self.status_topic = str(
            self.declare_parameter("status_topic", "/status").value
        )

        if self.mass <= 0.0:
            raise ValueError("mass must be positive")
        if self.gravity <= 0.0:
            raise ValueError("gravity must be positive")
        if not 0.0 < self.equilibrium_throttle <= 1.0:
            raise ValueError("equilibrium_throttle must be in (0, 1]")
        if not 0.0 <= self.min_throttle < self.max_throttle <= 1.0:
            raise ValueError("throttle limits must satisfy 0 <= min < max <= 1")
        if not 0.0 < self.min_projection_cosine <= 1.0:
            raise ValueError("min_projection_cosine must be in (0, 1]")

        self.hover_thrust = self.mass * self.gravity
        self.state = None
        self.rc_override = 0
        self.last_logged_override = 0
        self.publisher = self.create_publisher(
            ControllerCommand, self.output_topic, 1
        )
        self.subscription = self.create_subscription(
            ControllerCommand, self.input_topic, self.command_callback, 1
        )
        self.state_subscription = self.create_subscription(
            State, self.state_topic, self.state_callback, 1
        )
        self.status_subscription = self.create_subscription(
            Status, self.status_topic, self.status_callback, 1
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

    def state_callback(self, msg):
        self.state = msg

    def status_callback(self, msg):
        self.rc_override = msg.rc_override
        if self.rc_override == self.last_logged_override:
            return

        roll_overridden = bool(self.rc_override & self.ROLL_OVERRIDE_MASK)
        pitch_overridden = bool(self.rc_override & self.PITCH_OVERRIDE_MASK)
        if roll_overridden or pitch_overridden:
            self.get_logger().warn(
                "RC attitude takeover: mask=0x%03x roll=%s pitch=%s; "
                "using measured attitude for thrust projection"
                % (self.rc_override, roll_overridden, pitch_overridden)
            )
        elif self.last_logged_override & (
            self.ROLL_OVERRIDE_MASK | self.PITCH_OVERRIDE_MASK
        ):
            self.get_logger().info(
                "RC attitude takeover released; using limited offboard attitude "
                "for thrust projection"
            )
        self.last_logged_override = self.rc_override

    def invalidate(self, output, reason):
        output.cmd_valid = False
        self.publisher.publish(output)
        self.get_logger().error(f"{reason}; publishing an invalid command")

    def projection_attitude(self, limited_roll, limited_pitch):
        roll_overridden = bool(self.rc_override & self.ROLL_OVERRIDE_MASK)
        pitch_overridden = bool(self.rc_override & self.PITCH_OVERRIDE_MASK)
        if not (roll_overridden or pitch_overridden):
            return limited_roll, limited_pitch

        if self.state is None:
            self.get_logger().warn(
                "RC attitude takeover is active but no estimated state is available; "
                "using limited offboard attitude for thrust projection"
            )
            return limited_roll, limited_pitch

        projection_roll = self.state.phi if roll_overridden else limited_roll
        projection_pitch = self.state.theta if pitch_overridden else limited_pitch
        return projection_roll, projection_pitch

    def command_callback(self, msg):
        output = ControllerCommand()
        output.header = msg.header
        output.phi_c = msg.phi_c
        output.theta_c = msg.theta_c

        expected_mode = (
            ControllerCommand.MODE_ROLL_PITCH_YAWRATE_THRUST_TO_MIXER
        )
        if msg.mode != expected_mode:
            self.invalidate(
                output,
                f"Expected ControllerCommand mode {expected_mode}, got {msg.mode}",
            )
            return

        command_values = (msg.cmd1, msg.cmd2, msg.cmd3, msg.cmd4)
        if not all(math.isfinite(value) for value in command_values):
            self.invalidate(output, "Non-finite attitude or thrust command")
            return
        if msg.cmd4 < 0.0:
            self.invalidate(output, f"Invalid negative thrust command {msg.cmd4}")
            return

        limited_roll = max(-self.max_roll, min(self.max_roll, msg.cmd1))
        limited_pitch = max(-self.max_pitch, min(self.max_pitch, msg.cmd2))

        # The trajectory follower's cmd4 is the magnitude of a coherent 3-D
        # thrust vector. Preserve that vector's vertical component if its
        # roll/pitch direction is limited. During an RC axis takeover, use the
        # measured attitude for that axis because the firmware is no longer
        # applying the corresponding offboard attitude command.
        vertical_thrust = msg.cmd4 * math.cos(msg.cmd1) * math.cos(msg.cmd2)
        projection_roll, projection_pitch = self.projection_attitude(
            limited_roll, limited_pitch
        )
        applied_projection = math.cos(projection_roll) * math.cos(projection_pitch)

        if not math.isfinite(applied_projection) or applied_projection <= 0.0:
            self.invalidate(
                output,
                "Cannot project thrust through roll=%.3f pitch=%.3f"
                % (projection_roll, projection_pitch),
            )
            return

        applied_projection = max(applied_projection, self.min_projection_cosine)
        corrected_thrust = max(0.0, vertical_thrust) / applied_projection
        throttle = (
            corrected_thrust / self.hover_thrust * self.equilibrium_throttle
        )

        output.mode = ControllerCommand.MODE_ROLL_PITCH_YAWRATE_THROTTLE
        output.cmd1 = limited_roll
        output.cmd2 = limited_pitch
        output.cmd3 = max(-self.max_yaw_rate, min(self.max_yaw_rate, msg.cmd3))
        output.cmd4 = max(self.min_throttle, min(self.max_throttle, throttle))
        output.phi_c = limited_roll
        output.theta_c = limited_pitch
        output.cmd_valid = msg.cmd_valid
        self.publisher.publish(output)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= 1_000_000_000:
            self.last_log_ns = now_ns
            self.get_logger().info(
                "thrust=%.3f N vertical=%.3f N corrected=%.3f N -> "
                "throttle=%.4f, attitude=(%.3f, %.3f), projection "
                "attitude=(%.3f, %.3f), yaw_rate=%.3f valid=%s"
                % (
                    msg.cmd4,
                    vertical_thrust,
                    corrected_thrust,
                    output.cmd4,
                    output.cmd1,
                    output.cmd2,
                    projection_roll,
                    projection_pitch,
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
