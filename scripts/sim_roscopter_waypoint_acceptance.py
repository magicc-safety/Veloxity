#!/usr/bin/env python3
"""Launch Voloxide standalone sim plus ROScopter and verify waypoint response.

Run from an environment with Jazzy, the ROSflight workspace, this overlay, and
RMW_IMPLEMENTATION=rmw_zenoh_cpp sourced/exported.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from roscopter_msgs.msg import ControllerCommand, TrajectoryCommand, Waypoint
from roscopter_msgs.srv import AddWaypoint
from rosflight_msgs.msg import Command, PwmOutput, RCRaw, SimState, Status
from rosflight_msgs.srv import ParamFile, ParamGet
from std_srvs.srv import Trigger


OFFBOARD_RC = [1500, 1500, 1000, 1500, 2000, 1000, 1500, 1500]
DISARM_RC = [1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500]
TARGET = (4.0, 0.0, -3.0)
TUTORIAL_WAYPOINTS = [
    {
        "type": 1,
        "w": (0.0, 0.0, -10.0),
        "speed": 4.0,
        "psi": 0.0,
        "hold_seconds": 0.0,
        "hold_indefinitely": False,
        "use_lla": False,
    },
    {
        "type": 1,
        "w": (20.0, 0.0, -10.0),
        "speed": 4.0,
        "psi": 0.0,
        "hold_seconds": 0.0,
        "hold_indefinitely": False,
        "use_lla": False,
    },
    {
        "type": 0,
        "w": (20.0, -20.0, -20.0),
        "speed": 4.0,
        "psi": 0.0,
        "hold_seconds": 5.0,
        "hold_indefinitely": False,
        "use_lla": False,
    },
    {
        "type": 1,
        "w": (0.0, -20.0, -20.0),
        "speed": 4.0,
        "psi": 0.0,
        "hold_seconds": 0.0,
        "hold_indefinitely": False,
        "use_lla": False,
    },
]


def quiet_popen(args: list[str]) -> subprocess.Popen:
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)


class WaypointProbe(Node):
    def __init__(self) -> None:
        super().__init__("sim_roscopter_waypoint_acceptance")
        self.rc_pub = self.create_publisher(RCRaw, "/sim/RC", 10)
        self.truth: SimState | None = None
        self.status: Status | None = None
        self.command: Command | None = None
        self.trajectory_command: TrajectoryCommand | None = None
        self.high_level_command: ControllerCommand | None = None
        self.waypoint_count = 0
        self.trajectory_count = 0
        self.high_level_count = 0
        self.command_count = 0
        self.max_pwm_delta = 0
        self.create_subscription(SimState, "/sim/truth_state", self._truth_cb, 10)
        self.create_subscription(Status, "/status", self._status_cb, 10)
        self.create_subscription(Command, "/command", self._command_cb, 10)
        self.create_subscription(PwmOutput, "/sim/pwm_output", self._pwm_cb, 10)
        self.create_subscription(Waypoint, "/waypoints", self._waypoint_cb, 10)
        self.create_subscription(TrajectoryCommand, "/trajectory_command", self._trajectory_cb, 10)
        self.create_subscription(ControllerCommand, "/high_level_command", self._high_level_cb, 10)
        self.param_load = self.create_client(ParamFile, "/param_load_from_file")
        self.param_get = self.create_client(ParamGet, "/param_get")
        self.all_params_received = self.create_client(Trigger, "/all_params_received")
        self.calibrate = self.create_client(Trigger, "/calibrate_imu")
        self.param_write = self.create_client(Trigger, "/param_write")
        self.clear_waypoints = self.create_client(Trigger, "/path_planner/clear_waypoints")
        self.add_waypoint = self.create_client(AddWaypoint, "/path_planner/add_waypoint")

    def _truth_cb(self, msg: SimState) -> None:
        self.truth = msg

    def _status_cb(self, msg: Status) -> None:
        self.status = msg

    def _command_cb(self, msg: Command) -> None:
        self.command = msg
        self.command_count += 1

    def _pwm_cb(self, msg: PwmOutput) -> None:
        values = list(msg.values[:4])
        if values:
            self.max_pwm_delta = max(self.max_pwm_delta, max(abs(int(v) - 1000) for v in values))

    def _waypoint_cb(self, msg: Waypoint) -> None:
        if not msg.clear_wp_list:
            self.waypoint_count += 1

    def _trajectory_cb(self, msg: TrajectoryCommand) -> None:
        self.trajectory_command = msg
        self.trajectory_count += 1

    def _high_level_cb(self, msg: ControllerCommand) -> None:
        self.high_level_command = msg
        self.high_level_count += 1

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout_s: float) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.truth is not None and self.status is not None:
                return
        raise RuntimeError("timed out waiting for /status and /sim/truth_state")

    def call_trigger(self, client, name: str, timeout_s: float = 30.0) -> None:
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"{name} service not available")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        result = future.result() if future.done() else None
        if result is None or not result.success:
            message = getattr(result, "message", "") if result is not None else ""
            raise RuntimeError(f"{name} failed {message}")

    def initialize_firmware(self) -> None:
        if not self.param_load.wait_for_service(timeout_sec=45.0):
            raise RuntimeError("/param_load_from_file service not available")
        if not self.param_get.wait_for_service(timeout_sec=45.0):
            raise RuntimeError("/param_get service not available")
        if not self.all_params_received.wait_for_service(timeout_sec=45.0):
            raise RuntimeError("/all_params_received service not available")
        if not self.calibrate.wait_for_service(timeout_sec=45.0):
            raise RuntimeError("/calibrate_imu service not available")

        self.wait_all_params_received()

        param_file = os.path.join(
            get_package_share_directory("rosflight_sim"),
            "params",
            "multirotor_firmware",
            "multirotor_combined.yaml",
        )
        req = ParamFile.Request()
        req.filename = param_file
        future = self.param_load.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=45.0)
        result = future.result() if future.done() else None
        if result is None or not result.success:
            raise RuntimeError(f"failed to load params from {param_file}")

        self.wait_param_value("RC_ARM_CHN", 4.0)
        self.wait_param_value("RC_THR_OVRD_CHN", 5.0)
        self.wait_param_value("PRI_MIXER_OUT_0", 2.0)
        self.call_trigger(self.calibrate, "/calibrate_imu")
        self.call_trigger(self.param_write, "/param_write")

    def publish_rc_for(self, values: list[int], duration_s: float) -> None:
        msg = RCRaw()
        msg.values = values
        end = time.time() + duration_s
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.rc_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.015)

    def arm_and_publish_offboard_rc_until_ready(self, timeout_s: float) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.publish_rc_for(OFFBOARD_RC, 0.25)
            if (
                self.status is not None
                and self.status.armed
                and not self.status.failsafe
            ):
                return
        raise RuntimeError("timed out waiting for armed status")

    def wait_all_params_received(self) -> None:
        deadline = time.time() + 45.0
        while time.time() < deadline:
            future = self.all_params_received.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() and future.result().success:
                return
            self.spin_for(0.25)
        raise RuntimeError("timed out waiting for initial parameter sync")

    def wait_param_value(self, name: str, expected: float) -> None:
        deadline = time.time() + 45.0
        while time.time() < deadline:
            req = ParamGet.Request()
            req.name = name
            future = self.param_get.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            result = future.result() if future.done() else None
            if result is not None and result.exists and abs(result.value - expected) < 0.001:
                return
            self.spin_for(0.25)
        raise RuntimeError(f"timed out waiting for {name}={expected}")

    def add_target_waypoint(self) -> None:
        self.add_waypoints([self.make_goto_waypoint(TARGET, speed=2.0)])

    @staticmethod
    def make_goto_waypoint(target: tuple[float, float, float], speed: float) -> dict:
        return {
            "type": 1,
            "w": target,
            "speed": speed,
            "psi": 0.0,
            "hold_seconds": 0.0,
            "hold_indefinitely": False,
            "use_lla": False,
        }

    def add_waypoints(self, waypoints: list[dict]) -> None:
        self.call_trigger(self.clear_waypoints, "/path_planner/clear_waypoints", timeout_s=45.0)
        if not self.add_waypoint.wait_for_service(timeout_sec=45.0):
            raise RuntimeError("/path_planner/add_waypoint service not available")
        for index, waypoint in enumerate(waypoints):
            req = AddWaypoint.Request()
            req.wp.type = waypoint["type"]
            req.wp.w = list(waypoint["w"])
            req.wp.speed = waypoint["speed"]
            req.wp.psi = waypoint["psi"]
            req.wp.hold_seconds = waypoint["hold_seconds"]
            req.wp.hold_indefinitely = waypoint["hold_indefinitely"]
            req.wp.use_lla = waypoint["use_lla"]
            req.publish_now = index == 0
            future = self.add_waypoint.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
            result = future.result() if future.done() else None
            if result is None or not result.success:
                message = getattr(result, "message", "") if result is not None else ""
                raise RuntimeError(f"/path_planner/add_waypoint failed {message}")

    def distance_to_target(self) -> float:
        return self.distance_to(TARGET)

    def distance_to(self, target: tuple[float, float, float]) -> float:
        if self.truth is None:
            raise RuntimeError("missing truth-state sample")
        p = self.truth.pose.position
        return math.sqrt((target[0] - p.x) ** 2 + (target[1] - p.y) ** 2 + (target[2] - p.z) ** 2)

    def run_observation(self, duration_s: float) -> tuple[float, float, float, bool]:
        start_dist = self.distance_to_target()
        max_command_thrust = 0.0
        min_dist = start_dist
        deadline = time.time() + duration_s
        while time.time() < deadline:
            self.publish_rc_for(OFFBOARD_RC, 0.15)
            min_dist = min(min_dist, self.distance_to_target())
            if self.command is not None:
                max_command_thrust = max(max_command_thrust, abs(self.command.u[2]))
        end_dist = self.distance_to_target()
        status_ok = (
            self.status is not None
            and self.status.armed
            and not self.status.failsafe
        )
        command_ok = max_command_thrust > 0.1 and self.max_pwm_delta > 25
        return start_dist, end_dist, max_command_thrust, status_ok and command_ok

    def run_multi_observation(
        self,
        waypoints: list[tuple[float, float, float]],
        duration_s: float,
        tolerance_m: float,
    ) -> tuple[list[float], list[float], float, bool]:
        start_distances = [self.distance_to(waypoint) for waypoint in waypoints]
        min_distances = list(start_distances)
        max_command_thrust = 0.0
        deadline = time.time() + duration_s
        while time.time() < deadline:
            self.publish_rc_for(OFFBOARD_RC, 0.15)
            for index, waypoint in enumerate(waypoints):
                min_distances[index] = min(min_distances[index], self.distance_to(waypoint))
            if self.command is not None:
                max_command_thrust = max(max_command_thrust, abs(self.command.u[2]))

        status_ok = (
            self.status is not None
            and self.status.armed
            and not self.status.failsafe
            and self.status.offboard
            and self.status.rc_override == 0
        )
        command_ok = max_command_thrust > 0.1 and self.max_pwm_delta > 25
        reached_all = all(distance <= tolerance_m for distance in min_distances)
        return start_distances, min_distances, max_command_thrust, status_ok and command_ok and reached_all


def start_lower_processes(use_rviz: bool) -> list[subprocess.Popen]:
    return [
        quiet_popen(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]),
        quiet_popen(
            [
                "ros2",
                "launch",
                "voloxide_sil_board_shim",
                "multirotor_standalone_voloxide.launch.py",
                "use_builtin_rc:=false",
                f"use_rviz:={'true' if use_rviz else 'false'}",
            ]
        ),
    ]


def start_roscopter_processes() -> list[subprocess.Popen]:
    return [
        quiet_popen(
            [
                "ros2",
                "launch",
                "roscopter",
                "roscopter.launch.py",
                "state_topic:=sim/roscopter/state",
            ]
        ),
        quiet_popen(["ros2", "run", "roscopter_gcs", "rviz_waypoint_publisher"]),
        quiet_popen(
            [
                "ros2",
                "run",
                "roscopter_sim",
                "sim_state_transcriber",
                "--ros-args",
                "-r",
                "__node:=roscopter_truth",
            ]
        ),
    ]


def stop_processes(processes: list[subprocess.Popen]) -> None:
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    deadline = time.time() + 10.0
    for process in reversed(processes):
        while process.poll() is None and time.time() < deadline:
            time.sleep(0.1)
        if process.poll() is None:
            process.terminate()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--use-rviz", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--observe-seconds", type=float, default=30.0)
    parser.add_argument("--mission", choices=["single", "four"], default="single")
    parser.add_argument("--waypoint-tolerance", type=float, default=1.25)
    parser.add_argument("--keep-running", action="store_true")
    args = parser.parse_args()

    processes = start_lower_processes(args.use_rviz)
    rclpy.init()
    node = WaypointProbe()
    try:
        node.wait_ready(60.0)
        node.initialize_firmware()
        node.arm_and_publish_offboard_rc_until_ready(25.0)
        processes.extend(start_roscopter_processes())
        waypoints = [node.make_goto_waypoint(TARGET, speed=2.0)] if args.mission == "single" else TUTORIAL_WAYPOINTS
        waypoint_targets = [waypoint["w"] for waypoint in waypoints]
        if args.mission == "single":
            node.add_target_waypoint()
        else:
            node.add_waypoints(waypoints)
        node.spin_for(1.0)
        if node.command is None:
            raise RuntimeError("/command not observed after waypoint publication")

        if args.mission == "single":
            start_dist, end_dist, max_thrust, passed = node.run_observation(args.observe_seconds)
            print(f"target_ned={TARGET}")
            print(f"distance_start={start_dist:.3f} distance_end={end_dist:.3f}")
        else:
            start_distances, min_distances, max_thrust, passed = node.run_multi_observation(
                waypoint_targets,
                args.observe_seconds,
                args.waypoint_tolerance,
            )
            print(f"targets_ned={waypoint_targets}")
            print(
                "waypoint_start_distances="
                f"{[round(distance, 3) for distance in start_distances]}"
            )
            print(
                "waypoint_min_distances="
                f"{[round(distance, 3) for distance in min_distances]} "
                f"tolerance={args.waypoint_tolerance:.3f}"
            )
        print(f"max_command_thrust={max_thrust:.3f}")
        if node.status is not None:
            print(
                "status: "
                f"armed={node.status.armed} failsafe={node.status.failsafe} "
                f"offboard={node.status.offboard} control_mode={node.status.control_mode} "
                f"rc_override={node.status.rc_override} error_code={node.status.error_code}"
            )
        print(
            "roscopter_counts: "
            f"waypoints={node.waypoint_count} trajectory={node.trajectory_count} "
            f"high_level={node.high_level_count} command={node.command_count}"
        )
        if node.high_level_command is not None:
            print(
                "last_high_level: "
                f"mode={node.high_level_command.mode} valid={node.high_level_command.cmd_valid} "
                f"cmd=({node.high_level_command.cmd1:.3f},{node.high_level_command.cmd2:.3f},"
                f"{node.high_level_command.cmd3:.3f},{node.high_level_command.cmd4:.3f})"
            )
        print(f"max_pwm_delta={node.max_pwm_delta}")
        if node.command is not None:
            print(
                "last_command: "
                f"mode={node.command.mode} ignore={node.command.ignore} "
                f"u0_3={[round(v, 3) for v in node.command.u[:4]]}"
            )
        print("ROSCOPTER WAYPOINT RESPONSE OK" if passed else "ROSCOPTER WAYPOINT RESPONSE FAILED")
        return 0 if passed else 1
    finally:
        node.publish_rc_for(DISARM_RC, 0.5)
        node.destroy_node()
        rclpy.shutdown()
        if not args.keep_running:
            stop_processes(processes)


if __name__ == "__main__":
    sys.exit(main())
