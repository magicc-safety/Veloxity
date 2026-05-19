#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rosflight_msgs.msg import Command, PwmOutput, RCRaw, SimState, Status
from rosflight_msgs.srv import ParamFile, ParamGet, ParamSet, SetSimState
from std_srvs.srv import Trigger


REPO_ROOT = Path(__file__).resolve().parents[1]
ROSFLIGHT_SETUP = Path("/home/skink/projects/rosflight_setup/workspace/install/setup.zsh")
LAUNCH_FILE = REPO_ROOT / "ros2/voloxide_sil_board_shim/launch/multirotor_standalone_sil.launch.py"
TARGET_DIR = REPO_ROOT / "target"
ROS_LOG_DIR = TARGET_DIR / "ros2/roslog"
ROSBAG_DIR = TARGET_DIR / "rosbags"

ARM_RC = [1500, 1500, 1000, 1500, 2000, 2000, 1500, 1500]
OFFBOARD_RC = [1500, 1500, 1000, 1500, 2000, 1000, 1500, 1500]
DISARM_RC = [1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500]
NEUTRAL_RC = [1500, 1500, 1450, 1500, 2000, 2000, 1500, 1500]


def make_env() -> dict[str, str]:
    env = os.environ.copy()
    ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)
    ROSBAG_DIR.mkdir(parents=True, exist_ok=True)
    env["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    os.environ["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    env.setdefault("PYTHONUNBUFFERED", "1")
    return env


def require_ros_environment() -> None:
    make_env()
    if not Path("/opt/ros/jazzy/setup.zsh").exists():
        raise RuntimeError("/opt/ros/jazzy is missing")
    if not ROSFLIGHT_SETUP.exists():
        raise RuntimeError(f"ROSflight workspace setup file is missing: {ROSFLIGHT_SETUP}")
    if "rosflight_sim" not in os.environ.get("AMENT_PREFIX_PATH", ""):
        raise RuntimeError(
            "ROSflight packages are not sourced. Run:\n"
            "  source /opt/ros/jazzy/setup.zsh\n"
            f"  source {ROSFLIGHT_SETUP}"
        )


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
    for process in reversed(processes):
        if process.poll() is None:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()


@dataclass
class LaunchConfig:
    firmware: str = "c"
    use_builtin_rc: bool = True
    use_vimfly: bool = False
    use_rviz: bool = True
    use_roscopter: bool = False


def launch_stack(config: LaunchConfig) -> list[subprocess.Popen]:
    env = make_env()
    processes: list[subprocess.Popen] = []
    if env.get("RMW_IMPLEMENTATION") == "rmw_zenoh_cpp":
        processes.append(subprocess.Popen(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"], env=env))
        time.sleep(1.0)
    processes.append(subprocess.Popen([
        "ros2",
        "launch",
        str(LAUNCH_FILE),
        f"firmware:={config.firmware}",
        f"use_builtin_rc:={'true' if config.use_builtin_rc else 'false'}",
        f"use_vimfly:={'true' if config.use_vimfly else 'false'}",
        f"use_rviz:={'true' if config.use_rviz else 'false'}",
    ], env=env))
    if config.use_roscopter:
        time.sleep(4.0)
        processes.append(subprocess.Popen([
            "ros2",
            "launch",
            "roscopter",
            "roscopter.launch.py",
            "state_topic:=sim/roscopter/state",
        ], env=env))
        processes.append(subprocess.Popen([
            "ros2",
            "run",
            "roscopter_gcs",
            "rviz_waypoint_publisher",
        ], env=env))
        processes.append(subprocess.Popen([
            "ros2",
            "run",
            "roscopter_sim",
            "sim_state_transcriber",
            "--ros-args",
            "-r",
            "__node:=roscopter_truth",
        ], env=env))
    return processes


def start_bag(name: str, topics: list[str]) -> subprocess.Popen:
    env = make_env()
    output = ROSBAG_DIR / f"{name}-{time.strftime('%Y%m%d-%H%M%S')}"
    return subprocess.Popen(["ros2", "bag", "record", "-o", str(output), *topics], env=env)


class SilProbe(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.rc_pub = self.create_publisher(RCRaw, "/sim/RC", 10)
        self.command_pub = self.create_publisher(Command, "/command", 10)
        self.truth: SimState | None = None
        self.status: Status | None = None
        self.pwm: PwmOutput | None = None
        self.last_command: Command | None = None
        self.seen_armed = False
        self.seen_offboard = False
        self.max_pwm_delta = 0
        self.create_subscription(SimState, "/sim/truth_state", self._truth_cb, 10)
        self.create_subscription(Status, "/status", self._status_cb, 10)
        self.create_subscription(PwmOutput, "/sim/pwm_output", self._pwm_cb, 10)
        self.create_subscription(Command, "/command", self._command_cb, 10)
        self.param_load = self.create_client(ParamFile, "/param_load_from_file")
        self.param_get = self.create_client(ParamGet, "/param_get")
        self.param_set = self.create_client(ParamSet, "/param_set")
        self.all_params_received = self.create_client(Trigger, "/all_params_received")
        self.calibrate = self.create_client(Trigger, "/calibrate_imu")
        self.param_write = self.create_client(Trigger, "/param_write")
        self.toggle_arm = self.create_client(Trigger, "/toggle_arm")
        self.toggle_override = self.create_client(Trigger, "/toggle_override")
        self.set_sim_state = self.create_client(SetSimState, "/dynamics/set_sim_state")

    def _truth_cb(self, msg: SimState) -> None:
        self.truth = msg

    def _status_cb(self, msg: Status) -> None:
        self.status = msg
        self.seen_armed = self.seen_armed or msg.armed
        self.seen_offboard = self.seen_offboard or msg.offboard

    def _pwm_cb(self, msg: PwmOutput) -> None:
        self.pwm = msg
        values = list(msg.values[:4])
        if values:
            self.max_pwm_delta = max(self.max_pwm_delta, max(abs(int(v) - 1000) for v in values))

    def _command_cb(self, msg: Command) -> None:
        self.last_command = msg

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout_s: float = 60.0) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.truth is not None and self.status is not None:
                return
        raise RuntimeError("timed out waiting for /status and /sim/truth_state")

    def call_trigger(self, client, name: str, timeout_s: float = 30.0) -> None:
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"{name} service is not available")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        result = future.result() if future.done() else None
        if result is None or not result.success:
            raise RuntimeError(f"{name} failed: {getattr(result, 'message', '')}")

    def wait_armed(self, timeout_s: float = 8.0) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.status is not None and self.status.armed:
                return
        raise RuntimeError("timed out waiting for armed status")

    def wait_offboard(self, timeout_s: float = 8.0) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.status is not None and self.status.offboard:
                return
        raise RuntimeError("timed out waiting for offboard status")

    def initialize_firmware(self) -> None:
        for client, name in [
            (self.param_load, "/param_load_from_file"),
            (self.param_get, "/param_get"),
            (self.all_params_received, "/all_params_received"),
            (self.calibrate, "/calibrate_imu"),
        ]:
            if not client.wait_for_service(timeout_sec=45.0):
                raise RuntimeError(f"{name} service is not available")
        self.wait_all_params_received()
        req = ParamFile.Request()
        req.filename = os.path.join(
            get_package_share_directory("rosflight_sim"),
            "params",
            "multirotor_firmware",
            "multirotor_combined.yaml",
        )
        future = self.param_load.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=45.0)
        result = future.result() if future.done() else None
        if result is None or not result.success:
            raise RuntimeError(f"failed to load firmware params from {req.filename}")
        self.wait_param_value("RC_ARM_CHN", 4.0)
        self.wait_param_value("RC_THR_OVRD_CHN", 5.0)
        self.wait_param_value("FAILSAFE_THR", 0.0)
        self.call_trigger(self.calibrate, "/calibrate_imu")
        self.wait_any_imu_bias(timeout_s=30.0)

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

    def get_param_value(self, name: str, timeout_s: float = 2.0) -> float | None:
        req = ParamGet.Request()
        req.name = name
        future = self.param_get.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        result = future.result() if future.done() else None
        if result is not None and result.exists:
            return float(result.value)
        return None

    def set_param_value(self, name: str, value: float, timeout_s: float = 5.0) -> None:
        if not self.param_set.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("/param_set service is not available")
        req = ParamSet.Request()
        req.name = name
        req.value = value
        future = self.param_set.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        result = future.result() if future.done() else None
        if result is None or not result.exists:
            raise RuntimeError(f"failed to set {name}={value}")

    def wait_any_imu_bias(self, timeout_s: float) -> None:
        names = [
            "ACC_X_BIAS",
            "ACC_Y_BIAS",
            "ACC_Z_BIAS",
            "GYRO_X_BIAS",
            "GYRO_Y_BIAS",
            "GYRO_Z_BIAS",
        ]
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            for name in names:
                value = self.get_param_value(name, timeout_s=1.0)
                if value is not None and abs(value) > 1e-6:
                    self.spin_for(0.5)
                    return
            self.spin_for(0.25)
        raise RuntimeError("IMU calibration did not produce a nonzero accel or gyro bias")

    def publish_rc_for(self, values: list[int], duration_s: float) -> None:
        msg = RCRaw()
        msg.values = values
        end = time.time() + duration_s
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.rc_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.015)

    def publish_command_for(self, mode: int, values: list[float], duration_s: float) -> None:
        msg = Command()
        msg.mode = mode
        msg.ignore = Command.IGNORE_NONE
        for index, value in enumerate(values[:10]):
            msg.u[index] = value
        end = time.time() + duration_s
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.command_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.02)

    def arm_with_rc(self) -> None:
        deadline = time.time() + 8.0
        while time.time() < deadline:
            self.publish_rc_for(ARM_RC, 0.25)
            if self.status is not None and self.status.armed:
                return
        raise RuntimeError("timed out waiting for armed status")

    def sample_truth(self) -> SimState:
        self.spin_for(0.25)
        if self.truth is None:
            raise RuntimeError("missing /sim/truth_state sample")
        return self.truth

    def reset_sim_state(self) -> None:
        if not self.set_sim_state.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/dynamics/set_sim_state service is not available")
        req = SetSimState.Request()
        req.state.pose.orientation.w = 1.0
        future = self.set_sim_state.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result() if future.done() else None
        if result is None or not result.success:
            raise RuntimeError(f"/dynamics/set_sim_state failed: {getattr(result, 'message', '')}")
        self.spin_for(0.5)


def distance_to(truth: SimState, point: tuple[float, float, float]) -> float:
    pos = truth.pose.position
    return math.sqrt((pos.x - point[0]) ** 2 + (pos.y - point[1]) ** 2 + (pos.z - point[2]) ** 2)


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--firmware", choices=["c", "voloxide"], default="c")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--keep-running", action="store_true")
