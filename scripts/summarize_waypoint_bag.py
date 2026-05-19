#!/usr/bin/env python3
"""Summarize waypoint acceptance rosbag behavior."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TARGET = (4.0, 0.0, -3.0)


def distance_to_target(msg) -> float:
    p = msg.pose.position
    return math.sqrt((TARGET[0] - p.x) ** 2 + (TARGET[1] - p.y) ** 2 + (TARGET[2] - p.z) ** 2)


def summarize_bag(path: Path) -> None:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    counts: dict[str, int] = {}
    start_dist: float | None = None
    min_dist: float | None = None
    end_dist: float | None = None
    last_status = None
    last_command = None
    max_abs_thrust = 0.0
    max_pwm_delta = 0
    command_modes: set[int] = set()
    command_ignores: set[int] = set()
    high_level_modes: set[int] = set()
    last_high_level = None
    last_trajectory = None
    rviz_wp_markers = 0
    rviz_text_markers = 0
    rviz_clear_markers = 0

    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == "/sim/truth_state":
            distance = distance_to_target(msg)
            if start_dist is None:
                start_dist = distance
            min_dist = distance if min_dist is None else min(min_dist, distance)
            end_dist = distance
        elif topic == "/status":
            last_status = msg
        elif topic == "/command":
            last_command = msg
            command_modes.add(int(msg.mode))
            command_ignores.add(int(msg.ignore))
            if len(msg.u) >= 3:
                max_abs_thrust = max(max_abs_thrust, abs(float(msg.u[2])))
        elif topic == "/sim/pwm_output":
            values = list(msg.values[:4])
            if values:
                max_pwm_delta = max(max_pwm_delta, max(abs(int(value) - 1000) for value in values))
        elif topic == "/high_level_command":
            last_high_level = msg
            high_level_modes.add(int(msg.mode))
        elif topic == "/trajectory_command":
            last_trajectory = msg
        elif topic == "/rviz/waypoint":
            if int(msg.action) == 3:
                rviz_clear_markers += 1
            elif msg.ns == "wp":
                rviz_wp_markers += 1
            elif msg.ns == "text":
                rviz_text_markers += 1

    print(path)
    print(f"  target_ned={TARGET}")
    print(
        "  distance: "
        f"start={start_dist:.3f} min={min_dist:.3f} end={end_dist:.3f}"
        if start_dist is not None and min_dist is not None and end_dist is not None
        else "  distance: no /sim/truth_state samples"
    )
    print(f"  counts: {', '.join(f'{topic}={count}' for topic, count in sorted(counts.items()))}")
    if last_status is not None:
        print(
            "  status: "
            f"armed={last_status.armed} failsafe={last_status.failsafe} "
            f"offboard={last_status.offboard} control_mode={last_status.control_mode} "
            f"rc_override={last_status.rc_override} error_code={last_status.error_code}"
        )
    if last_command is not None:
        print(
            "  command: "
            f"modes={sorted(command_modes)} ignores={sorted(command_ignores)} "
            f"max_abs_thrust={max_abs_thrust:.3f} "
            f"last_u={[round(float(value), 3) for value in last_command.u[:4]]}"
        )
    print(f"  max_pwm_delta={max_pwm_delta}")
    if last_high_level is not None:
        print(
            "  high_level: "
            f"modes={sorted(high_level_modes)} last_valid={last_high_level.cmd_valid} "
            f"last=({last_high_level.cmd1:.3f},{last_high_level.cmd2:.3f},"
            f"{last_high_level.cmd3:.3f},{last_high_level.cmd4:.3f})"
        )
    if last_trajectory is not None:
        print(
            "  trajectory: "
            f"last_pos=({last_trajectory.position[0]:.3f},{last_trajectory.position[1]:.3f},"
            f"{last_trajectory.position[2]:.3f})"
        )
    if "/rviz/waypoint" in counts:
        print(
            "  rviz_waypoint_markers: "
            f"wp={rviz_wp_markers} text={rviz_text_markers} clear={rviz_clear_markers}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    args = parser.parse_args()

    for bag in args.bags:
        summarize_bag(bag)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
