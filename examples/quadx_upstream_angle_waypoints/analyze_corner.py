#!/usr/bin/env python3
"""Report position and velocity at a waypoint corner from a ROS 2 bag."""

import argparse
import bisect
import math
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def database(path):
    path = Path(path)
    if path.is_file():
        return path
    matches = sorted(path.glob("*.db3"))
    if not matches:
        raise SystemExit(f"No .db3 file found in {path}")
    return matches[0]


def load(cur, topics, name):
    topic_id, type_name = topics[name]
    message_type = get_message(type_name)
    return [
        (stamp, deserialize_message(data, message_type))
        for stamp, data in cur.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp",
            (topic_id,),
        )
    ]


def nearest(rows, target):
    stamps = [stamp for stamp, _ in rows]
    index = bisect.bisect_left(stamps, target)
    candidates = rows[max(0, index - 1) : index + 1]
    return min(candidates, key=lambda row: abs(row[0] - target))


def truth_velocity(truth, stamp, interval_ns=250_000_000):
    before_stamp, before = nearest(truth, stamp - interval_ns)
    after_stamp, after = nearest(truth, stamp + interval_ns)
    dt = (after_stamp - before_stamp) / 1e9
    return (
        (after.pose.position.x - before.pose.position.x) / dt,
        (after.pose.position.y - before.pose.position.y) / dt,
    )


def horizontal_estimator_velocity(state):
    cosine = math.cos(state.psi)
    sine = math.sin(state.psi)
    return (
        cosine * state.v_x - sine * state.v_y,
        sine * state.v_x + cosine * state.v_y,
    )


def vector_string(vector):
    return f"({vector[0]:+.2f}, {vector[1]:+.2f}) m/s"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--north", type=float, default=20.0)
    parser.add_argument("--east", type=float, default=0.0)
    parser.add_argument("--capture-radius", type=float, default=0.15)
    args = parser.parse_args()

    connection = sqlite3.connect(str(database(args.bag)))
    cursor = connection.cursor()
    topics = {
        name: (topic_id, type_name)
        for topic_id, name, type_name in cursor.execute(
            "select id, name, type from topics"
        )
    }
    required = ["/status", "/sim/truth_state", "/estimated_state", "/trajectory_command"]
    missing = [name for name in required if name not in topics]
    if missing:
        raise SystemExit(f"Missing topics: {', '.join(missing)}")

    status = load(cursor, topics, "/status")
    truth = load(cursor, topics, "/sim/truth_state")
    estimate = load(cursor, topics, "/estimated_state")
    trajectory = load(cursor, topics, "/trajectory_command")
    release = next(
        stamp for stamp, message in status if message.armed and message.rc_override == 0
    )
    _, truth_at_release = nearest(truth, release)
    _, estimate_at_release = nearest(estimate, release)
    truth_origin_offset = (
        estimate_at_release.p_n - truth_at_release.pose.position.x,
        estimate_at_release.p_e - truth_at_release.pose.position.y,
        estimate_at_release.p_d - truth_at_release.pose.position.z,
    )

    def aligned_truth_position(state):
        return (
            state.pose.position.x + truth_origin_offset[0],
            state.pose.position.y + truth_origin_offset[1],
            state.pose.position.z + truth_origin_offset[2],
        )

    waypoint = (args.north, args.east)
    captures = [
        (stamp, message)
        for stamp, message in trajectory
        if stamp >= release
        and math.hypot(message.position[0] - waypoint[0], message.position[1] - waypoint[1])
        <= args.capture_radius
    ]
    if not captures:
        raise SystemExit("Trajectory never entered the requested waypoint capture radius")
    capture_stamp, command = captures[0]

    prior_stamp = max(release, capture_stamp - 2_000_000_000)
    _, prior_command = nearest(trajectory, prior_stamp)
    approach = (
        waypoint[0] - prior_command.position[0],
        waypoint[1] - prior_command.position[1],
    )
    norm = math.hypot(*approach)
    approach = (approach[0] / norm, approach[1] / norm)

    end_stamp = capture_stamp + 8_000_000_000
    for stamp, message in trajectory:
        if stamp <= capture_stamp:
            continue
        if math.hypot(message.position[0] - waypoint[0], message.position[1] - waypoint[1]) > 0.25:
            end_stamp = stamp
            break

    corner_truth = [(stamp, message) for stamp, message in truth if capture_stamp <= stamp <= end_stamp]
    overshoot_stamp, overshoot_state = max(
        corner_truth,
        key=lambda row: (
            (aligned_truth_position(row[1])[0] - waypoint[0]) * approach[0]
            + (aligned_truth_position(row[1])[1] - waypoint[1]) * approach[1]
        ),
    )
    closest_stamp, closest_state = min(
        corner_truth,
        key=lambda row: math.hypot(
            aligned_truth_position(row[1])[0] - waypoint[0],
            aligned_truth_position(row[1])[1] - waypoint[1],
        ),
    )

    _, truth_at_capture = nearest(truth, capture_stamp)
    _, estimate_at_capture = nearest(estimate, capture_stamp)
    _, estimate_at_overshoot = nearest(estimate, overshoot_stamp)
    truth_v = truth_velocity(truth, capture_stamp)
    estimate_v = horizontal_estimator_velocity(estimate_at_capture)
    truth_position = aligned_truth_position(truth_at_capture)[:2]
    estimator_error = math.hypot(
        estimate_at_capture.p_n - truth_position[0],
        estimate_at_capture.p_e - truth_position[1],
    )
    overshoot = (
        (aligned_truth_position(overshoot_state)[0] - waypoint[0]) * approach[0]
        + (aligned_truth_position(overshoot_state)[1] - waypoint[1]) * approach[1]
    )
    estimated_overshoot = max(
        (message.p_n - waypoint[0]) * approach[0]
        + (message.p_e - waypoint[1]) * approach[1]
        for stamp, message in estimate
        if capture_stamp <= stamp <= end_stamp
    )

    print(f"Waypoint: north={waypoint[0]:.2f}, east={waypoint[1]:.2f}")
    print(
        "Truth-to-estimator origin alignment at release: "
        f"({truth_origin_offset[0]:+.2f}, {truth_origin_offset[1]:+.2f}, "
        f"{truth_origin_offset[2]:+.2f}) m"
    )
    print(f"Trajectory capture: {(capture_stamp - release) / 1e9:.2f} s after release")
    print(
        f"  commanded position=({command.position[0]:+.2f}, {command.position[1]:+.2f}) m, "
        f"velocity={vector_string(command.velocity)}"
    )
    print(
        f"  truth position=({truth_position[0]:+.2f}, {truth_position[1]:+.2f}) m, "
        f"velocity={vector_string(truth_v)}"
    )
    print(
        f"  estimate position=({estimate_at_capture.p_n:+.2f}, {estimate_at_capture.p_e:+.2f}) m, "
        f"velocity={vector_string(estimate_v)}"
    )
    print(f"  estimator horizontal error={estimator_error:.2f} m")
    print(
        f"Closest aligned-truth pass: {(closest_stamp - capture_stamp) / 1e9:+.2f} s, "
        f"position=({aligned_truth_position(closest_state)[0]:+.2f}, "
        f"{aligned_truth_position(closest_state)[1]:+.2f}) m"
    )
    print(
        f"Maximum aligned-truth along-track overshoot: {overshoot:.2f} m at "
        f"{(overshoot_stamp - capture_stamp) / 1e9:+.2f} s"
    )
    print(
        f"  truth position=({aligned_truth_position(overshoot_state)[0]:+.2f}, "
        f"{aligned_truth_position(overshoot_state)[1]:+.2f}) m, "
        f"estimate=({estimate_at_overshoot.p_n:+.2f}, {estimate_at_overshoot.p_e:+.2f}) m"
    )
    print(f"Maximum estimated along-track overshoot: {estimated_overshoot:.2f} m")


if __name__ == "__main__":
    main()
