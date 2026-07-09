#!/usr/bin/env python3
"""Temporary waypoint-to-angle controller for quad-X firmware tests.

This node keeps ROScopter's estimator/path planner/path manager in the loop,
but replaces the default thrust-to-mixer final stage. It consumes
roscopter_msgs/TrajectoryCommand and roscopter_msgs/State, then publishes
rosflight_msgs/Command in MODE_ROLL_PITCH_YAWRATE_THROTTLE.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError

from roscopter_msgs.msg import State, TrajectoryCommand
from rosflight_msgs.msg import Command, Status


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TrajectoryToAngleCommand(Node):
    def __init__(self):
        super().__init__("trajectory_to_angle_command_experiment")
        self.command_pub = self.create_publisher(Command, "/command", 10)
        self.state_sub = self.create_subscription(
            State, "/estimated_state", self.state_callback, 10
        )
        self.traj_sub = self.create_subscription(
            TrajectoryCommand, "/trajectory_command", self.traj_callback, 10
        )
        self.status_sub = self.create_subscription(Status, "/status", self.status_callback, 10)

        self.state = None
        self.traj = None
        self.status = None
        self.last_time = self.get_clock().now()
        self.filtered_n = None
        self.filtered_e = None
        self.filtered_d = None
        self.filtered_psi = 0.0
        self.filtered_vn = 0.0
        self.filtered_ve = 0.0
        self.filtered_vd = 0.0
        self.timer = self.create_timer(1.0 / 100.0, self.publish_command)
        self.log_timer = self.create_timer(0.5, self.log_state)

        self.hover_throttle = self.declare_parameter("hover_throttle", 0.686).value
        self.min_throttle = self.declare_parameter("min_throttle", 0.43).value
        self.max_throttle = self.declare_parameter("max_throttle", 0.82).value
        self.max_angle = self.declare_parameter("max_angle_rad", 0.30).value
        self.max_yaw_rate = self.declare_parameter("max_yaw_rate_rad_s", 0.7).value
        self.use_filtered_target = self.declare_parameter("use_filtered_target", False).value
        self.max_horizontal_slew = self.declare_parameter("max_horizontal_slew_m_s", 2.0).value
        self.max_down_slew = self.declare_parameter("max_down_slew_m_s", 1.0).value
        self.takeoff_attitude_gate_m = self.declare_parameter("takeoff_attitude_gate_m", 0.30).value
        self.max_horizontal_position_error = self.declare_parameter(
            "max_horizontal_position_error_m", 3.0
        ).value
        self.max_vertical_position_error = self.declare_parameter(
            "max_vertical_position_error_m", 2.0
        ).value

        self.kp_n = self.declare_parameter("kp_n", 0.55).value
        self.kd_n = self.declare_parameter("kd_n", 0.85).value
        self.kp_e = self.declare_parameter("kp_e", 0.55).value
        self.kd_e = self.declare_parameter("kd_e", 0.85).value
        self.max_horizontal_accel = self.declare_parameter("max_horizontal_accel_m_s2", 2.5).value

        self.kp_d = self.declare_parameter("kp_d_throttle", 0.040).value
        self.kd_d = self.declare_parameter("kd_d_throttle", 0.080).value
        self.kff_d_accel = self.declare_parameter("kff_d_accel_throttle", -0.010).value
        self.yaw_kp = self.declare_parameter("yaw_kp", 1.2).value
        self.yaw_slew_rate = self.declare_parameter("yaw_slew_rate_rad_s", 0.7).value

        self.get_logger().info(
            "Converting /trajectory_command to ROSflight angle/throttle /command"
        )

    def state_callback(self, msg):
        self.state = msg

    def traj_callback(self, msg):
        self.traj = msg

    def status_callback(self, msg):
        self.status = msg

    def publish_command(self):
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds * 1e-9, 1e-3)
        self.last_time = now

        msg = Command()
        msg.header.stamp = now.to_msg()
        msg.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
        msg.ignore = Command.IGNORE_NONE

        if self.state is None or self.traj is None:
            msg.u[2] = 0.0
            self.command_pub.publish(msg)
            return

        s = self.state
        t = self.traj

        if self.filtered_n is None:
            self.reset_filtered_target(s)

        psi = s.psi
        cpsi = math.cos(psi)
        spsi = math.sin(psi)

        # ROScopter state velocities are body-frame in the C++ trajectory follower.
        vn = cpsi * s.v_x - spsi * s.v_y
        ve = spsi * s.v_x + cpsi * s.v_y
        vd = s.v_z

        if self.status is None or self.status.rc_override != 0 or not self.status.armed:
            self.reset_filtered_target(s)
        elif self.use_filtered_target:
            self.slew_filtered_target(t, dt)

        if self.use_filtered_target:
            pn_cmd = self.filtered_n
            pe_cmd = self.filtered_e
            pd_cmd = self.filtered_d
            vn_cmd = self.filtered_vn
            ve_cmd = self.filtered_ve
            vd_cmd = self.filtered_vd
        else:
            pn_cmd = t.position[0]
            pe_cmd = t.position[1]
            pd_cmd = t.position[2]
            vn_cmd = t.velocity[0]
            ve_cmd = t.velocity[1]
            vd_cmd = t.velocity[2]
        an_ff, ae_ff, ad_ff = t.acceleration

        pn_cmd, pe_cmd = self.clamp_horizontal_reference(s, pn_cmd, pe_cmd)
        pd_cmd = s.p_d + clamp(
            pd_cmd - s.p_d,
            -self.max_vertical_position_error,
            self.max_vertical_position_error,
        )

        # Simple trajectory controller: desired inertial acceleration.
        an = self.kp_n * (pn_cmd - s.p_n) + self.kd_n * (vn_cmd - vn) + an_ff
        ae = self.kp_e * (pe_cmd - s.p_e) + self.kd_e * (ve_cmd - ve) + ae_ff

        an = clamp(an, -self.max_horizontal_accel, self.max_horizontal_accel)
        ae = clamp(ae, -self.max_horizontal_accel, self.max_horizontal_accel)

        # Convert inertial horizontal acceleration to body roll/pitch angles.
        forward_accel = cpsi * an + spsi * ae
        right_accel = -spsi * an + cpsi * ae
        theta = clamp(-forward_accel / 9.80665, -self.max_angle, self.max_angle)
        phi = clamp(right_accel / 9.80665, -self.max_angle, self.max_angle)

        # Normalized throttle from NED altitude error. Positive pd error means
        # the vehicle is above the command; negative vd means climbing.
        pd_error_up = s.p_d - pd_cmd
        throttle = (
            self.hover_throttle
            + self.kp_d * pd_error_up
            + self.kd_d * (vd - vd_cmd)
            + self.kff_d_accel * ad_ff
        )
        throttle = clamp(throttle, self.min_throttle, self.max_throttle)

        yaw_rate = clamp(
            self.yaw_kp * wrap_pi(self.filtered_psi - psi) + t.psi_dot,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )

        # Keep attitude commands masked while on the ground, mirroring ROScopter's
        # takeoff protection but without invoking the mixer-passthrough path.
        if abs(s.p_d) < self.takeoff_attitude_gate_m:
            phi = 0.0
            theta = 0.0
            yaw_rate = 0.0

        msg.u[0] = 0.0
        msg.u[1] = 0.0
        msg.u[2] = float(throttle)
        msg.u[3] = float(phi)
        msg.u[4] = float(theta)
        msg.u[5] = float(yaw_rate)
        self.command_pub.publish(msg)

    def reset_filtered_target(self, state):
        self.filtered_n = state.p_n
        self.filtered_e = state.p_e
        self.filtered_d = state.p_d
        self.filtered_psi = state.psi
        self.filtered_vn = 0.0
        self.filtered_ve = 0.0
        self.filtered_vd = 0.0

    def slew_filtered_target(self, traj, dt):
        dn = traj.position[0] - self.filtered_n
        de = traj.position[1] - self.filtered_e
        dd = traj.position[2] - self.filtered_d

        h_dist = math.hypot(dn, de)
        max_h_step = self.max_horizontal_slew * dt
        if h_dist > max_h_step and h_dist > 1e-6:
            scale = max_h_step / h_dist
            n_step = dn * scale
            e_step = de * scale
        else:
            n_step = dn
            e_step = de

        max_d_step = self.max_down_slew * dt
        d_step = clamp(dd, -max_d_step, max_d_step)

        self.filtered_n += n_step
        self.filtered_e += e_step
        self.filtered_d += d_step
        self.filtered_psi += clamp(
            wrap_pi(traj.psi - self.filtered_psi),
            -self.yaw_slew_rate * dt,
            self.yaw_slew_rate * dt,
        )

        self.filtered_vn = n_step / dt
        self.filtered_ve = e_step / dt
        self.filtered_vd = d_step / dt

    def clamp_horizontal_reference(self, state, pn_cmd, pe_cmd):
        dn = pn_cmd - state.p_n
        de = pe_cmd - state.p_e
        distance = math.hypot(dn, de)
        if distance <= self.max_horizontal_position_error or distance < 1e-6:
            return pn_cmd, pe_cmd
        scale = self.max_horizontal_position_error / distance
        return state.p_n + dn * scale, state.p_e + de * scale

    def log_state(self):
        if self.state is None or self.traj is None:
            self.get_logger().info("Waiting for /estimated_state and /trajectory_command")
            return
        s = self.state
        t = self.traj
        self.get_logger().info(
            "state=({:.1f},{:.1f},{:.1f}) filtered=({:.1f},{:.1f},{:.1f}) raw=({:.1f},{:.1f},{:.1f}) psi={:.2f}".format(
                s.p_n,
                s.p_e,
                s.p_d,
                self.filtered_n if self.filtered_n is not None else 0.0,
                self.filtered_e if self.filtered_e is not None else 0.0,
                self.filtered_d if self.filtered_d is not None else 0.0,
                t.position[0],
                t.position[1],
                t.position[2],
                s.psi,
            )
        )


def main():
    rclpy.init()
    node = TrajectoryToAngleCommand()
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
