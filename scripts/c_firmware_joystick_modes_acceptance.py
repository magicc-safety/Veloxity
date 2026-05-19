#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

import rclpy

from sil_test_lib import (
    ARM_RC,
    LaunchConfig,
    SilProbe,
    add_common_args,
    launch_stack,
    require_ros_environment,
    stop_processes,
)


TEST_NEUTRAL_RC = [1500, 1500, 1250, 1500, 2000, 2000, 1500, 1500]


@dataclass
class Case:
    name: str
    rc: list[int]
    axis: str
    sign: int


CASES = [
    Case("angle_pitch_forward", [1500, 2000, 1250, 1500, 2000, 2000, 1500, 1500], "vx", -1),
    Case("angle_pitch_back", [1500, 1000, 1250, 1500, 2000, 2000, 1500, 1500], "vx", 1),
    Case("angle_roll_right", [1000, 1500, 1250, 1500, 2000, 2000, 1500, 1500], "vy", -1),
    Case("angle_roll_left", [2000, 1500, 1250, 1500, 2000, 2000, 1500, 1500], "vy", 1),
    Case("rate_yaw_cw", [1500, 1500, 1250, 1000, 2000, 2000, 1500, 1500], "wz", -1),
    Case("rate_yaw_ccw", [1500, 1500, 1250, 2000, 2000, 2000, 1500, 1500], "wz", 1),
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify joystick-like RC angle and yaw-rate behavior.")
    add_common_args(parser)
    args = parser.parse_args()
    require_ros_environment()

    processes = launch_stack(LaunchConfig(
        firmware=args.firmware,
        use_builtin_rc=False,
        use_rviz=not args.no_rviz,
    ))
    rclpy.init()
    node = SilProbe("c_firmware_joystick_modes_acceptance")
    try:
        node.wait_ready()
        node.initialize_firmware()
        node.publish_rc_for(ARM_RC, 2.0)
        node.arm_with_rc()
        node.publish_rc_for(TEST_NEUTRAL_RC, 2.0)
        failures = []
        for case in CASES:
            node.reset_sim_state()
            node.publish_rc_for(TEST_NEUTRAL_RC, 0.75)
            before = node.sample_truth()
            node.publish_rc_for(case.rc, 1.0)
            after = node.sample_truth()
            node.publish_rc_for(TEST_NEUTRAL_RC, 0.75)
            dvx = after.twist.linear.x - before.twist.linear.x
            dvy = after.twist.linear.y - before.twist.linear.y
            dwz = after.twist.angular.z - before.twist.angular.z
            value = {"vx": dvx, "vy": dvy, "wz": dwz}[case.axis]
            line = f"{case.name}: dv=({dvx:.3f},{dvy:.3f}) dwz={dwz:.3f}"
            if not math.isfinite(value) or value * case.sign <= 0.04:
                failures.append(line)
                print("FAIL " + line)
            else:
                print("PASS " + line)
        if failures:
            raise RuntimeError("; ".join(failures))
        return 0
    except Exception as exc:
        print(f"FAIL joystick modes: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if not args.keep_running:
            stop_processes(processes)


if __name__ == "__main__":
    raise SystemExit(main())
TEST_NEUTRAL_RC = [1500, 1500, 1250, 1500, 2000, 2000, 1500, 1500]
