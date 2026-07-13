#!/usr/bin/env python3
"""Report vertical estimator/truth divergence and barometer calibration events."""

from __future__ import annotations

import argparse
import bisect
import math
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def database(path: Path) -> Path:
    if path.is_file():
        return path
    matches = sorted(path.glob("*.db3"))
    if not matches:
        raise SystemExit(f"No .db3 file found in {path}")
    return matches[0]


def load(cursor, topics, name):
    topic_id, type_name = topics[name]
    message_type = get_message(type_name)
    return [
        (stamp, deserialize_message(data, message_type))
        for stamp, data in cursor.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp", (topic_id,)
        )
    ]


def nearest(rows, target):
    stamps = [stamp for stamp, _ in rows]
    index = bisect.bisect_left(stamps, target)
    return min(rows[max(0, index - 1):index + 1], key=lambda row: abs(row[0] - target))


def nearest_with_stamps(rows, stamps, target):
    index = bisect.bisect_left(stamps, target)
    return min(rows[max(0, index - 1):index + 1], key=lambda row: abs(row[0] - target))


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def analyze(path: Path) -> None:
    connection = sqlite3.connect(str(database(path)))
    cursor = connection.cursor()
    topics = {
        name: (topic_id, type_name)
        for topic_id, name, type_name in cursor.execute("select id, name, type from topics")
    }
    status = load(cursor, topics, "/status")
    truth = load(cursor, topics, "/sim/truth_state")
    estimate = load(cursor, topics, "/estimated_state")
    trajectory = load(cursor, topics, "/trajectory_command")
    baro = load(cursor, topics, "/baro")
    rosout = load(cursor, topics, "/rosout")

    armed = next(stamp for stamp, msg in status if msg.armed)
    release = next(stamp for stamp, msg in status if msg.armed and msg.rc_override == 0)
    _, truth_release = nearest(truth, release)
    _, estimate_release = nearest(estimate, release)
    down_offset = estimate_release.p_d - truth_release.pose.position.z

    errors = []
    truth_command_errors = []
    estimate_command_errors = []
    final = None
    truth_stamps = [stamp for stamp, _ in truth]
    trajectory_stamps = [stamp for stamp, _ in trajectory]
    for stamp, state in estimate[::10]:
        if not (release <= stamp <= release + 120_000_000_000):
            continue
        _, truth_state = nearest_with_stamps(truth, truth_stamps, stamp)
        _, command = nearest_with_stamps(trajectory, trajectory_stamps, stamp)
        aligned_truth_down = truth_state.pose.position.z + down_offset
        errors.append(state.p_d - aligned_truth_down)
        truth_command_errors.append(aligned_truth_down - command.position[2])
        estimate_command_errors.append(state.p_d - command.position[2])
        final = (state.p_d, aligned_truth_down, command.position[2])

    calibration_baro = [msg.pressure for stamp, msg in baro if armed <= stamp <= armed + 3_000_000_000]
    pressure_span = max(calibration_baro) - min(calibration_baro)
    events = [
        ((stamp - armed) / 1e9, msg.msg)
        for stamp, msg in rosout
        if "baro calibration" in msg.msg.lower()
    ]

    print(path)
    print(f"  release {((release - armed) / 1e9):.2f} s after arm; down alignment={down_offset:+.3f} m")
    print(f"  estimate-minus-aligned-truth down RMS={rms(errors):.3f} m max={max(map(abs, errors)):.3f} m")
    print(f"  aligned-truth-minus-command down RMS={rms(truth_command_errors):.3f} m")
    print(f"  estimate-minus-command down RMS={rms(estimate_command_errors):.3f} m")
    if final is not None:
        print(f"  final down estimate/truth/command={final[0]:+.2f}/{final[1]:+.2f}/{final[2]:+.2f} m")
    print(f"  first 3 s after arm baro pressure span={pressure_span:.2f} Pa over {len(calibration_baro)} samples")
    if events:
        for offset, event in events:
            print(f"  baro event at arm{offset:+.2f}s: {event}")
    else:
        print("  no baro calibration warning in recorded /rosout")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    args = parser.parse_args()
    for bag in args.bags:
        analyze(bag)


if __name__ == "__main__":
    main()
