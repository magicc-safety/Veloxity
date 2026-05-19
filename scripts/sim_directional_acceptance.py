#!/usr/bin/env python3
"""Launch ROSflight standalone sim and verify Voloxide directional signs.

Run from an environment with Jazzy, the ROSflight workspace, this overlay, and
a ROS 2 RMW implementation sourced/exported.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rosflight_msgs.msg import PwmOutput, RCRaw, SimState, Status
from rosflight_msgs.srv import ParamFile, ParamGet
from std_srvs.srv import Trigger


NEUTRAL = [1500, 1500, 1450, 1500, 2000, 2000, 1500, 1500]
ARM = [1500, 1500, 1000, 1500, 2000, 2000, 1500, 1500]
DISARM = [1500, 1500, 1000, 1500, 1000, 2000, 1500, 1500]


@dataclass
class Case:
    name: str
    values: list[int]
    axis: str
    expected_sign: int


CASES = [
    Case("pitch_forward_ch1_2000", [1500, 2000, 1450, 1500, 2000, 2000, 1500, 1500], "vx", -1),
    Case("pitch_backward_ch1_1000", [1500, 1000, 1450, 1500, 2000, 2000, 1500, 1500], "vx", 1),
    Case("roll_right_ch0_1000", [1000, 1500, 1450, 1500, 2000, 2000, 1500, 1500], "vy", -1),
    Case("roll_left_ch0_2000", [2000, 1500, 1450, 1500, 2000, 2000, 1500, 1500], "vy", 1),
    Case("yaw_cw_ch3_1000", [1500, 1500, 1450, 1000, 2000, 2000, 1500, 1500], "wz", -1),
    Case("yaw_ccw_ch3_2000", [1500, 1500, 1450, 2000, 2000, 2000, 1500, 1500], "wz", 1),
]


class DirectionalProbe(Node):
    def __init__(self) -> None:
        super().__init__("sim_directional_acceptance")
        self.pub = self.create_publisher(RCRaw, "/sim/RC", 10)
        self.truth: SimState | None = None
        self.status: Status | None = None
        self.pwm: PwmOutput | None = None
        self.create_subscription(SimState, "/sim/truth_state", self._truth_cb, 10)
        self.create_subscription(Status, "/status", self._status_cb, 10)
        self.create_subscription(PwmOutput, "/sim/pwm_output", self._pwm_cb, 10)
        self.param_load = self.create_client(ParamFile, "/param_load_from_file")
        self.param_get = self.create_client(ParamGet, "/param_get")
        self.all_params_received = self.create_client(Trigger, "/all_params_received")
        self.calibrate = self.create_client(Trigger, "/calibrate_imu")

    def _truth_cb(self, msg: SimState) -> None:
        self.truth = msg

    def _status_cb(self, msg: Status) -> None:
        self.status = msg

    def _pwm_cb(self, msg: PwmOutput) -> None:
        self.pwm = msg

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

    def call_init_services(self) -> None:
        if not self.param_load.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/param_load_from_file service not available")
        if not self.param_get.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/param_get service not available")
        if not self.all_params_received.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/all_params_received service not available")
        if not self.calibrate.wait_for_service(timeout_sec=30.0):
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or not future.result() or not future.result().success:
            raise RuntimeError(f"failed to load params from {param_file}")

        self.wait_param_value("RC_ARM_CHN", 4.0)
        self.wait_param_value("PRI_MIXER_OUT_0", 2.0)

        future = self.calibrate.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or not future.result() or not future.result().success:
            raise RuntimeError("IMU calibration service failed")

    def arm(self) -> None:
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self.publish_for(ARM, 0.25)
            if self.status is not None and self.status.armed:
                return
        raise RuntimeError("timed out waiting for armed status")

    def wait_all_params_received(self) -> None:
        deadline = time.time() + 30.0
        while time.time() < deadline:
            future = self.all_params_received.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() and future.result().success:
                return
            self.spin_for(0.25)
        raise RuntimeError("timed out waiting for initial parameter sync")

    def wait_param_value(self, name: str, expected: float) -> None:
        deadline = time.time() + 30.0
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

    def publish_for(self, values: list[int], duration: float) -> None:
        msg = RCRaw()
        msg.values = values
        end = time.time() + duration
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.015)

    def sample(self) -> SimState:
        self.spin_for(0.25)
        if self.truth is None:
            raise RuntimeError("missing truth-state sample")
        return self.truth

    def run_case(self, case: Case) -> tuple[bool, str]:
        self.publish_for(NEUTRAL, 1.0)
        before = self.sample()
        self.publish_for(case.values, 2.0)
        after = self.sample()
        self.publish_for(NEUTRAL, 0.75)

        dvx = after.twist.linear.x - before.twist.linear.x
        dvy = after.twist.linear.y - before.twist.linear.y
        dwz = after.twist.angular.z - before.twist.angular.z
        value = {"vx": dvx, "vy": dvy, "wz": dwz}[case.axis]
        passed = value * case.expected_sign > 0.15
        line = (
            f"{case.name}: dv=({dvx:.3f},{dvy:.3f}) "
            f"dwz={dwz:.3f} expected {case.axis} sign {case.expected_sign:+d}"
        )
        return passed, line


def start_processes(baseline: str, use_rviz: bool) -> list[subprocess.Popen]:
    launch_file = (
        "multirotor_standalone_upstream_baseline.launch.py"
        if baseline == "upstream"
        else "multirotor_standalone_voloxide.launch.py"
    )
    processes = [
        subprocess.Popen(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]),
        subprocess.Popen(
            [
                "ros2",
                "launch",
                "voloxide_sil_board_shim",
                launch_file,
                "use_builtin_rc:=false" if baseline == "rust" else "use_sim_time:=false",
                f"use_rviz:={'true' if use_rviz else 'false'}",
            ]
        ),
    ]
    return processes


def stop_processes(processes: list[subprocess.Popen]) -> None:
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    deadline = time.time() + 8.0
    for process in reversed(processes):
        while process.poll() is None and time.time() < deadline:
            time.sleep(0.1)
        if process.poll() is None:
            process.terminate()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", choices=["rust", "upstream"], default="rust")
    parser.add_argument("--use-rviz", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--keep-running", action="store_true")
    args = parser.parse_args()

    processes = start_processes(args.baseline, args.use_rviz)
    rclpy.init()
    node = DirectionalProbe()
    passed = False
    try:
        node.wait_ready(45.0)
        node.call_init_services()
        node.arm()
        node.publish_for(NEUTRAL, 2.0)

        results = [node.run_case(case) for case in CASES]
        passed = all(result for result, _line in results)
        for result, line in results:
            print(("PASS " if result else "FAIL ") + line)

        node.publish_for(DISARM, 1.0)
        if node.status is not None:
            print(
                "status: "
                f"armed={node.status.armed} failsafe={node.status.failsafe} "
                f"error_code={node.status.error_code} rc_override={node.status.rc_override} "
                f"mode={node.status.control_mode}"
            )
        if node.pwm is not None:
            print(f"pwm: {list(node.pwm.values[:4])}")

        print(
            "SIGN CONVENTION OK: matches upstream ROSflight standalone behavior"
            if passed
            else "SIGN CONVENTION FAILED"
        )
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if not args.keep_running:
            stop_processes(processes)


if __name__ == "__main__":
    sys.exit(main())
