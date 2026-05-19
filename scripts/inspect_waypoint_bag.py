#!/usr/bin/env python3
"""Inspect time-aligned waypoint path and command samples from a rosbag."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TARGET = (4.0, 0.0, -3.0)


@dataclass
class Sample:
    t: float
    x: float
    y: float
    z: float
    dist: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float
    traj: object | None
    high: object | None
    cmd: object | None
    pwm: object | None
    status: object | None


def distance(x: float, y: float, z: float) -> float:
    return math.sqrt((TARGET[0] - x) ** 2 + (TARGET[1] - y) ** 2 + (TARGET[2] - z) ** 2)


def euler_from_quat(q) -> tuple[float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def read_samples(path: Path) -> tuple[list[Sample], dict[str, float]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    start_ns: int | None = None
    latest_traj = None
    latest_high = None
    latest_cmd = None
    latest_pwm = None
    latest_status = None
    previous_armed = False
    samples: list[Sample] = []
    event_times: dict[str, float] = {}
    status_events: list[tuple[float, int]] = []
    waypoint_events: list[tuple[float, tuple[float, float, float], bool]] = []
    trajectory_changes: list[tuple[float, tuple[float, float, float]]] = []
    last_traj_pos: tuple[float, float, float] | None = None

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if start_ns is None:
            start_ns = stamp
        t = (stamp - start_ns) / 1e9
        msg = deserialize_message(data, get_message(type_map[topic]))

        if topic == "/waypoints" and not msg.clear_wp_list and "first_waypoint" not in event_times:
            event_times["first_waypoint"] = t
        if topic == "/waypoints":
            waypoint_events.append((t, tuple(float(value) for value in msg.w), bool(msg.clear_wp_list)))
        elif topic == "/trajectory_command":
            latest_traj = msg
            traj_pos = tuple(round(float(value), 3) for value in msg.position)
            if last_traj_pos is None or distance(*traj_pos) > 0.01 and traj_pos != last_traj_pos:
                trajectory_changes.append((t, traj_pos))
                last_traj_pos = traj_pos
            if "first_trajectory" not in event_times:
                event_times["first_trajectory"] = t
        elif topic == "/high_level_command":
            latest_high = msg
            if "first_high_level" not in event_times:
                event_times["first_high_level"] = t
        elif topic == "/command":
            latest_cmd = msg
            if "first_command" not in event_times:
                event_times["first_command"] = t
        elif topic == "/sim/pwm_output":
            latest_pwm = msg
        elif topic == "/status":
            previous_error = getattr(latest_status, "error_code", 0) if latest_status is not None else 0
            latest_status = msg
            if msg.armed and not previous_armed and "first_armed" not in event_times:
                event_times["first_armed"] = t
            if previous_armed and not msg.armed and "first_disarmed_after_arm" not in event_times:
                event_times["first_disarmed_after_arm"] = t
            previous_armed = bool(msg.armed)
            if previous_error != msg.error_code:
                status_events.append((t, int(msg.error_code)))
                if previous_error == 0 and msg.error_code != 0 and "first_error" not in event_times:
                    event_times["first_error"] = t
        elif topic == "/sim/truth_state":
            p = msg.pose.position
            v = msg.twist.linear
            roll, pitch, yaw = euler_from_quat(msg.pose.orientation)
            samples.append(
                Sample(
                    t=t,
                    x=p.x,
                    y=p.y,
                    z=p.z,
                    dist=distance(p.x, p.y, p.z),
                    vx=v.x,
                    vy=v.y,
                    vz=v.z,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    traj=latest_traj,
                    high=latest_high,
                    cmd=latest_cmd,
                    pwm=latest_pwm,
                    status=latest_status,
                )
            )

    if "first_waypoint" in event_times:
        waypoint_t = event_times["first_waypoint"]
        for t, error_code in status_events:
            if t >= waypoint_t and error_code != 0:
                event_times["first_error_after_waypoint"] = t
                break

    event_times["_waypoint_events"] = waypoint_events  # type: ignore[assignment]
    event_times["_trajectory_changes"] = trajectory_changes  # type: ignore[assignment]
    event_times["_status_events"] = status_events  # type: ignore[assignment]
    return samples, event_times


def nearest(samples: list[Sample], t: float) -> Sample:
    return min(samples, key=lambda sample: abs(sample.t - t))


def first_sustained_approach(samples: list[Sample], start_t: float) -> float:
    post = [sample for sample in samples if sample.t >= start_t]
    for index in range(0, max(0, len(post) - 50)):
        current = post[index]
        later = post[index + 50]
        if current.dist - later.dist > 0.5:
            return current.t
    return post[0].t if post else samples[0].t


def fmt_vec(values, count: int = 4) -> str:
    return "[" + ", ".join(f"{float(value):.3f}" for value in list(values)[:count]) + "]"


def print_sample(label: str, sample: Sample) -> None:
    status = sample.status
    cmd = sample.cmd
    high = sample.high
    traj = sample.traj
    pwm = sample.pwm
    print(label)
    print(
        f"  t={sample.t:.2f}s pos=({sample.x:.3f},{sample.y:.3f},{sample.z:.3f}) "
        f"dist={sample.dist:.3f} vel=({sample.vx:.3f},{sample.vy:.3f},{sample.vz:.3f}) "
        f"rpy_deg=({sample.roll:.1f},{sample.pitch:.1f},{sample.yaw:.1f})"
    )
    if status is not None:
        print(
            f"  status armed={status.armed} offboard={status.offboard} failsafe={status.failsafe} "
            f"rc_override={status.rc_override} error_code={status.error_code}"
        )
    if traj is not None:
        print(
            f"  trajectory pos={fmt_vec(traj.position, 3)} vel={fmt_vec(traj.velocity, 3)} "
            f"acc={fmt_vec(traj.acceleration, 3)} psi={traj.psi:.3f} psi_dot={traj.psi_dot:.3f}"
        )
    if high is not None:
        print(
            f"  high_level mode={high.mode} valid={high.cmd_valid} "
            f"cmd=({high.cmd1:.3f},{high.cmd2:.3f},{high.cmd3:.3f},{high.cmd4:.3f}) "
            f"phi_c={high.phi_c:.3f} theta_c={high.theta_c:.3f}"
        )
    if cmd is not None:
        print(f"  command mode={cmd.mode} ignore={cmd.ignore} u={fmt_vec(cmd.u, 10)}")
    if pwm is not None:
        print(f"  pwm first4={list(pwm.values[:4])}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()

    samples, event_times = read_samples(args.bag)
    if not samples:
        raise RuntimeError("no /sim/truth_state samples")

    waypoint_t = event_times.get("first_waypoint", event_times.get("first_command", samples[0].t))
    approach_t = first_sustained_approach(samples, waypoint_t)
    min_sample = min((sample for sample in samples if sample.t >= waypoint_t), key=lambda sample: sample.dist)
    final_sample = samples[-1]

    print(args.bag)
    waypoint_events = event_times.pop("_waypoint_events", [])  # type: ignore[assignment]
    trajectory_changes = event_times.pop("_trajectory_changes", [])  # type: ignore[assignment]
    status_events = event_times.pop("_status_events", [])  # type: ignore[assignment]

    print("events: " + ", ".join(f"{name}={t:.2f}s" for name, t in sorted(event_times.items(), key=lambda item: item[1])))
    print("waypoint_events:")
    for t, wp, clear in waypoint_events[:12]:
        print(f"  t={t:.2f}s clear={clear} w=({wp[0]:.3f},{wp[1]:.3f},{wp[2]:.3f})")
    print("status_error_changes:")
    for t, error_code in status_events[:20]:
        print(f"  t={t:.2f}s error_code={error_code}")
    print("trajectory_position_changes_near_waypoint:")
    for t, pos in trajectory_changes:
        if waypoint_t - 5.0 <= t <= waypoint_t + 8.0:
            print(f"  t={t:.2f}s pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})")
    print_sample("first waypoint command path sample", nearest(samples, waypoint_t))
    if "first_armed" in event_times:
        arm_t = event_times["first_armed"]
        print_sample("first armed sample", nearest(samples, arm_t))
        for offset in [0.25, 0.5, 1.0, 1.5, 2.0]:
            print_sample(f"path sample +{offset:.2f}s after arming", nearest(samples, arm_t + offset))
    print_sample("first sustained movement toward target", nearest(samples, approach_t))
    for offset in [2.0, 4.0, 6.0, 8.0, 10.0]:
        print_sample(f"path sample +{offset:.0f}s after sustained movement", nearest(samples, approach_t + offset))
    print_sample("minimum distance before divergence", min_sample)
    if "first_error" in event_times:
        print_sample("first nonzero status error", nearest(samples, event_times["first_error"]))
    print_sample("final sample", final_sample)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
