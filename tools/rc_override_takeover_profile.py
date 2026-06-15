#!/usr/bin/env python3
"""Publish a deterministic RC takeover/release profile for ROSflight SIL."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rosflight_msgs.msg import RCRaw, Status
from rosflight_msgs.srv import ParamFile
from std_srvs.srv import Trigger


DEFAULT_PARAM_FILE = (
    "/home/skink/projects/ROSflight/.distrobox-home/ROSflight/rosflight/workspace/src/"
    "rosflight_ros_pkgs/rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml"
)


@dataclass(frozen=True)
class RcPhase:
    name: str
    duration_s: float
    values: list[int]


class RcTakeoverProfile(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("veloxity_rc_override_takeover_profile")
        self.args = args
        self.pub = self.create_publisher(RCRaw, args.rc_topic, 1)
        self.status_sub = self.create_subscription(Status, "/status", self._status_cb, 10)
        self.latest_status: Status | None = None
        self.start_monotonic = time.monotonic()
        self.phase_start_monotonic = self.start_monotonic
        self.phase_index = -1

        neutral_offboard = self._channels(
            arm=2000,
            override=1000,
            throttle=args.neutral_throttle,
        )
        neutral_override_low_throttle = self._channels(
            arm=2000,
            override=2000,
            throttle=1000,
        )
        takeover = self._channels(
            arm=2000,
            override=2000,
            roll=args.roll_pwm,
            pitch=args.pitch_pwm,
            yaw=args.yaw_pwm,
            throttle=args.takeover_throttle,
        )
        self.phases = [
            RcPhase("prearm: override on, throttle low", args.prearm_s, neutral_override_low_throttle),
            RcPhase("offboard: override released", args.offboard_before_s, neutral_offboard),
            RcPhase("rc takeover: override on with stick command", args.takeover_s, takeover),
            RcPhase("offboard resume: override released", args.offboard_after_s, neutral_offboard),
        ]

        period_s = 1.0 / args.rate_hz
        self.timer = self.create_timer(period_s, self._tick)

    def _channels(
        self,
        *,
        roll: int = 1500,
        pitch: int = 1500,
        throttle: int = 1000,
        yaw: int = 1500,
        arm: int = 1000,
        override: int = 1000,
    ) -> list[int]:
        values = [1500] * 8
        values[0] = self._clamp_pwm(roll)
        values[1] = self._clamp_pwm(pitch)
        values[2] = self._clamp_pwm(throttle)
        values[3] = self._clamp_pwm(yaw)
        values[4] = self._clamp_pwm(arm)
        values[5] = self._clamp_pwm(override)
        return values

    @staticmethod
    def _clamp_pwm(value: int) -> int:
        return max(1000, min(2000, int(value)))

    def _status_cb(self, msg: Status) -> None:
        self.latest_status = msg

    def _publish(self, values: list[int]) -> None:
        msg = RCRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.values = values
        self.pub.publish(msg)

    def _tick(self) -> None:
        elapsed_s = time.monotonic() - self.start_monotonic
        cursor = 0.0
        selected_index = len(self.phases) - 1
        for idx, phase in enumerate(self.phases):
            cursor += phase.duration_s
            if elapsed_s <= cursor:
                selected_index = idx
                break

        if selected_index != self.phase_index:
            self.phase_index = selected_index
            self.phase_start_monotonic = time.monotonic()
            phase = self.phases[selected_index]
            self.get_logger().info(
                f"phase {selected_index + 1}/{len(self.phases)}: {phase.name}; "
                f"channels={phase.values}"
            )

        phase = self.phases[self.phase_index]
        self._publish(phase.values)

        total_duration_s = sum(item.duration_s for item in self.phases)
        if math.floor(elapsed_s) != math.floor(elapsed_s - (1.0 / self.args.rate_hz)):
            self._log_status(elapsed_s)

        if elapsed_s >= total_duration_s:
            self._publish(self.phases[-1].values)
            self._log_status(elapsed_s, final=True)
            rclpy.shutdown()

    def _log_status(self, elapsed_s: float, *, final: bool = False) -> None:
        if self.latest_status is None:
            self.get_logger().info(f"t={elapsed_s:.1f}s status: not received yet")
            return
        status = self.latest_status
        prefix = "final " if final else ""
        self.get_logger().info(
            f"{prefix}t={elapsed_s:.1f}s status: armed={status.armed} "
            f"failsafe={status.failsafe} rc_override={status.rc_override} "
            f"offboard={status.offboard} control_mode={status.control_mode} "
            f"error_code={status.error_code} loop_time_us={status.loop_time_us}"
        )


def call_trigger(node: Node, service_name: str, timeout_s: float) -> bool:
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        node.get_logger().error(f"service unavailable: {service_name}")
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done() or future.result() is None:
        node.get_logger().error(f"service timed out: {service_name}")
        return False
    result = future.result()
    node.get_logger().info(f"{service_name}: success={result.success} message={result.message!r}")
    return bool(result.success)


def call_param_load(node: Node, service_name: str, filename: str, timeout_s: float) -> bool:
    client = node.create_client(ParamFile, service_name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        node.get_logger().error(f"service unavailable: {service_name}")
        return False
    request = ParamFile.Request()
    request.filename = filename
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done() or future.result() is None:
        node.get_logger().error(f"service timed out: {service_name}")
        return False
    result = future.result()
    node.get_logger().info(f"{service_name}: success={result.success}")
    return bool(result.success)


def initialize_firmware(args: argparse.Namespace) -> bool:
    node = Node("veloxity_rc_takeover_firmware_init")
    try:
        ok = call_param_load(node, "/param_load_from_file", args.param_file, args.service_timeout_s)
        ok = call_trigger(node, "/calibrate_imu", args.service_timeout_s) and ok
        ok = call_trigger(node, "/calibrate_baro", args.service_timeout_s) and ok
        if args.write_params:
            time.sleep(args.write_delay_s)
            ok = call_trigger(node, "/param_write", args.service_timeout_s) and ok
        return ok
    finally:
        node.destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Drive a ROSflight SIL RC override/release profile on sim/RC."
    )
    parser.add_argument("--rc-topic", default="/sim/RC")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--param-file", default=DEFAULT_PARAM_FILE)
    parser.add_argument("--init-firmware", action="store_true")
    parser.add_argument("--write-params", action="store_true")
    parser.add_argument("--write-delay-s", type=float, default=2.0)
    parser.add_argument("--service-timeout-s", type=float, default=20.0)
    parser.add_argument("--prearm-s", type=float, default=2.0)
    parser.add_argument("--offboard-before-s", type=float, default=22.0)
    parser.add_argument("--takeover-s", type=float, default=3.0)
    parser.add_argument("--offboard-after-s", type=float, default=16.0)
    parser.add_argument("--neutral-throttle", type=int, default=1000)
    parser.add_argument("--takeover-throttle", type=int, default=1540)
    parser.add_argument("--roll-pwm", type=int, default=1500)
    parser.add_argument("--pitch-pwm", type=int, default=1515)
    parser.add_argument("--yaw-pwm", type=int, default=1500)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    try:
        if args.init_firmware and not initialize_firmware(args):
            raise SystemExit(1)
        node = RcTakeoverProfile(args)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
