#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

import rclpy
from rosflight_msgs.msg import Command

from sil_test_lib import LaunchConfig, SilProbe, add_common_args, launch_stack, require_ros_environment, stop_processes


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify tutorial-style C firmware arming services.")
    add_common_args(parser)
    args = parser.parse_args()
    require_ros_environment()

    processes = launch_stack(LaunchConfig(
        firmware=args.firmware,
        use_builtin_rc=True,
        use_rviz=not args.no_rviz,
    ))
    rclpy.init()
    node = SilProbe("c_firmware_arming_acceptance")
    try:
        node.wait_ready()
        node.initialize_firmware()
        node.call_trigger(node.toggle_arm, "/toggle_arm")
        node.spin_for(4.0)
        if not node.seen_armed:
            raise RuntimeError("status did not become armed after /toggle_arm")
        node.call_trigger(node.toggle_override, "/toggle_override")
        node.publish_command_for(
            Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE,
            [0.0, 0.0, 0.0, 0.0],
            2.0,
        )
        node.spin_for(2.0)
        if not node.seen_offboard:
            raise RuntimeError("status did not enter offboard after /toggle_override")
        print("PASS arming sequence: armed=true and offboard=true")
        return 0
    except Exception as exc:
        print(f"FAIL arming sequence: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if not args.keep_running:
            stop_processes(processes)


if __name__ == "__main__":
    raise SystemExit(main())
