#!/usr/bin/env python3
"""Wait until ROScopter estimated state agrees with sim truth."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rosflight_msgs.msg import SimState
from roscopter_msgs.msg import State


class ConvergenceGate:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.node = rclpy.create_node("roscopter_state_convergence_gate")
        self.estimate: State | None = None
        self.truth: SimState | None = None
        self.good_since: float | None = None
        self.last_error: tuple[float, float] | None = None
        self.node.create_subscription(State, args.estimated_topic, self.estimate_cb, 10)
        self.node.create_subscription(SimState, args.truth_topic, self.truth_cb, 10)

    def estimate_cb(self, msg: State) -> None:
        self.estimate = msg

    def truth_cb(self, msg: SimState) -> None:
        self.truth = msg

    def current_error(self) -> tuple[float, float] | None:
        if self.estimate is None or self.truth is None:
            return None
        pos = self.truth.pose.position
        horizontal = math.hypot(self.estimate.p_n - pos.x, self.estimate.p_e - pos.y)
        vertical = abs(self.estimate.p_d - pos.z)
        return horizontal, vertical

    def wait(self) -> bool:
        deadline = time.monotonic() + self.args.timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            error = self.current_error()
            if error is None:
                continue
            self.last_error = error
            horizontal, vertical = error
            converged = (
                horizontal <= self.args.horizontal_tolerance_m
                and vertical <= self.args.vertical_tolerance_m
            )
            now = time.monotonic()
            if converged:
                if self.good_since is None:
                    self.good_since = now
                if now - self.good_since >= self.args.stable_s:
                    return True
            else:
                self.good_since = None
        return False

    def close(self) -> None:
        self.node.destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimated-topic", default="/estimated_state")
    parser.add_argument("--truth-topic", default="/sim/truth_state")
    parser.add_argument("--horizontal-tolerance-m", type=float, default=1.0)
    parser.add_argument("--vertical-tolerance-m", type=float, default=1.0)
    parser.add_argument("--stable-s", type=float, default=1.0)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    gate = ConvergenceGate(args)
    try:
        success = gate.wait()
        if gate.last_error is None:
            print("state_error: unavailable")
        else:
            horizontal, vertical = gate.last_error
            print(f"state_error_horizontal_m: {horizontal:.3f}")
            print(f"state_error_vertical_m: {vertical:.3f}")
        print(f"converged: {str(success).lower()}")
        if not success:
            raise SystemExit(1)
    finally:
        gate.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
