#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

import rclpy
from rosflight_msgs.msg import Command

from sil_test_lib import ARM_RC, OFFBOARD_RC, LaunchConfig, SilProbe, add_common_args, launch_stack, require_ros_environment, stop_processes


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify direct /command passthrough changes C firmware PWM output.")
    add_common_args(parser)
    args = parser.parse_args()
    require_ros_environment()

    processes = launch_stack(LaunchConfig(
        firmware=args.firmware,
        use_builtin_rc=False,
        use_rviz=not args.no_rviz,
    ))
    rclpy.init()
    node = SilProbe("c_firmware_passthrough_acceptance")
    try:
        node.wait_ready()
        node.initialize_firmware()
        node.publish_rc_for(ARM_RC, 2.0)
        node.arm_with_rc()
        node.publish_rc_for(OFFBOARD_RC, 1.5)
        before = node.max_pwm_delta
        node.publish_rc_for(OFFBOARD_RC, 0.5)
        node.publish_command_for(
            Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE,
            [0.15, 0.0, 0.0, 0.55],
            3.0,
        )
        after = node.max_pwm_delta
        if node.status is None or not node.status.offboard:
            raise RuntimeError("firmware did not report offboard=true while /command was active")
        if after <= max(before + 20, 60):
            raise RuntimeError(f"PWM did not respond enough to passthrough command: before={before}, after={after}")
        print(f"PASS passthrough command: pwm_delta={after}, offboard=true")
        return 0
    except Exception as exc:
        print(f"FAIL passthrough command: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if not args.keep_running:
            stop_processes(processes)


if __name__ == "__main__":
    raise SystemExit(main())
