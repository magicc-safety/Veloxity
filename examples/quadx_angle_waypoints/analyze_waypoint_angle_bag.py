#!/usr/bin/env python3
"""Analyze quad-X waypoint angle-mode experiment bags."""

import argparse
import bisect
import math
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def resolve_db(path):
    path = Path(path)
    if path.is_dir():
        dbs = sorted(path.glob("*.db3"))
        if not dbs:
            raise SystemExit(f"No .db3 files found in {path}")
        return dbs[0]
    return path


def load_topic(cur, topic_meta, name):
    topic_id, type_name = topic_meta[name]
    cls = get_message(type_name)
    rows = []
    for stamp, data in cur.execute(
        "select timestamp, data from messages where topic_id=? order by timestamp",
        (topic_id,),
    ):
        rows.append((stamp, deserialize_message(data, cls)))
    return rows


def nearest(rows, stamps, target):
    idx = bisect.bisect_left(stamps, target)
    candidates = []
    if idx < len(rows):
        candidates.append(rows[idx])
    if idx:
        candidates.append(rows[idx - 1])
    if not candidates:
        return None
    return min(candidates, key=lambda item: abs(item[0] - target))


def percentile(values, pct):
    if not values:
        return float("nan")
    values = sorted(values)
    idx = min(len(values) - 1, max(0, round((pct / 100.0) * (len(values) - 1))))
    return values[idx]


def summarize(args):
    db = resolve_db(args.bag)
    con = sqlite3.connect(str(db))
    cur = con.cursor()
    topic_meta = {
        name: (topic_id, type_name)
        for topic_id, name, type_name in cur.execute("select id, name, type from topics")
    }

    required = [
        "/status",
        "/command",
        "/sim/pwm_output",
        "/sim/truth_state",
        "/estimated_state",
        "/trajectory_command",
    ]
    missing = [name for name in required if name not in topic_meta]
    if missing:
        raise SystemExit(f"Missing required topics: {', '.join(missing)}")

    status = load_topic(cur, topic_meta, "/status")
    command = load_topic(cur, topic_meta, "/command")
    pwm = load_topic(cur, topic_meta, "/sim/pwm_output")
    truth = load_topic(cur, topic_meta, "/sim/truth_state")
    estimate = load_topic(cur, topic_meta, "/estimated_state")
    trajectory = load_topic(cur, topic_meta, "/trajectory_command")

    truth_ts = [stamp for stamp, _ in truth]
    traj_ts = [stamp for stamp, _ in trajectory]
    est_ts = [stamp for stamp, _ in estimate]

    release = None
    arm = None
    for stamp, msg in status:
        if msg.armed and arm is None:
            arm = stamp
        if msg.armed and msg.rc_override == 0 and release is None:
            release = stamp
            break
    if release is None:
        release = truth[0][0]
        print("No armed rc_override=0 release found; using first truth timestamp.")

    start = min(rows[0][0] for rows in [status, command, truth, trajectory] if rows)
    end = max(rows[-1][0] for rows in [status, command, truth, trajectory] if rows)

    mode_counts = {}
    throttle_values = []
    roll_values = []
    pitch_values = []
    yaw_rate_values = []
    for stamp, msg in command:
        if stamp < release:
            continue
        mode_counts[msg.mode] = mode_counts.get(msg.mode, 0) + 1
        u = list(msg.u)
        throttle_values.append(u[2])
        roll_values.append(abs(u[3]))
        pitch_values.append(abs(u[4]))
        yaw_rate_values.append(abs(u[5]))

    motor_values = []
    saturation_count = 0
    for stamp, msg in pwm:
        if stamp < release:
            continue
        motors = list(msg.values[:4])
        motor_values.extend(motors)
        if max(motors) >= args.saturation_pwm or min(motors) <= args.low_pwm:
            saturation_count += 1

    lateral_errors = []
    vertical_errors = []
    estimator_vertical_errors = []
    for stamp, tr in truth:
        if stamp < release:
            continue
        near_traj = nearest(trajectory, traj_ts, stamp)
        if near_traj is None:
            continue
        _, traj = near_traj
        pos = tr.pose.position
        lateral_errors.append(
            math.hypot(pos.x - traj.position[0], pos.y - traj.position[1])
        )
        vertical_errors.append(pos.z - traj.position[2])

        near_est = nearest(estimate, est_ts, stamp)
        if near_est is not None:
            _, est = near_est
            estimator_vertical_errors.append(est.p_d - pos.z)

    print(f"Bag: {db.parent}")
    print(f"Duration: {(end - start) / 1e9:.1f} s")
    print(f"Release: {(release - start) / 1e9:.1f} s after bag start")
    print(f"Command modes after release: {mode_counts}")
    print(
        "Command ranges after release: "
        f"throttle=[{min(throttle_values):.3f}, {max(throttle_values):.3f}], "
        f"max|roll|={max(roll_values):.3f} rad, "
        f"max|pitch|={max(pitch_values):.3f} rad, "
        f"max|yaw_rate|={max(yaw_rate_values):.3f} rad/s"
    )
    print(
        "PWM after release: "
        f"range=[{min(motor_values)}, {max(motor_values)}], "
        f"saturation_samples={saturation_count}"
    )
    print(
        "Tracking error truth vs trajectory: "
        f"lateral mean={sum(lateral_errors)/len(lateral_errors):.2f} m, "
        f"lateral p95={percentile(lateral_errors, 95):.2f} m, "
        f"lateral max={max(lateral_errors):.2f} m"
    )
    print(
        "Altitude error truth.z - trajectory.z: "
        f"mean={sum(vertical_errors)/len(vertical_errors):.2f} m, "
        f"p95_abs={percentile([abs(v) for v in vertical_errors], 95):.2f} m, "
        f"max_abs={max(abs(v) for v in vertical_errors):.2f} m"
    )
    print(
        "Estimator altitude error est.p_d - truth.z: "
        f"mean={sum(estimator_vertical_errors)/len(estimator_vertical_errors):.2f} m, "
        f"p95_abs={percentile([abs(v) for v in estimator_vertical_errors], 95):.2f} m, "
        f"max_abs={max(abs(v) for v in estimator_vertical_errors):.2f} m"
    )

    print("Samples:")
    for seconds in args.samples:
        target = release + int(seconds * 1e9)
        near_truth = nearest(truth, truth_ts, target)
        near_traj = nearest(trajectory, traj_ts, target)
        near_cmd = nearest(command, [stamp for stamp, _ in command], target)
        if near_truth is None or near_traj is None or near_cmd is None:
            continue
        _, tr = near_truth
        _, traj = near_traj
        _, cmd = near_cmd
        pos = tr.pose.position
        u = list(cmd.u)
        print(
            f"  t={seconds:5.1f}s "
            f"truth=({pos.x:6.1f},{pos.y:6.1f},{pos.z:6.1f}) "
            f"traj=({traj.position[0]:6.1f},{traj.position[1]:6.1f},{traj.position[2]:6.1f}) "
            f"mode={cmd.mode} thr={u[2]:.3f} roll={u[3]:.3f} pitch={u[4]:.3f}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Bag directory or db3 file")
    parser.add_argument("--saturation-pwm", type=int, default=1900)
    parser.add_argument("--low-pwm", type=int, default=1050)
    parser.add_argument(
        "--samples",
        type=float,
        nargs="*",
        default=[0, 2, 5, 10, 20, 30, 40, 50, 60, 80],
    )
    summarize(parser.parse_args())


if __name__ == "__main__":
    main()
