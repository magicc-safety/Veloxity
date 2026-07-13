#!/usr/bin/env python3
"""Quantify the first divergence between SIL IMU input, firmware IMU, and attitude."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
from geometry_msgs.msg import Vector3Stamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosflight_msgs.msg import SimState
from sensor_msgs.msg import Imu


TOPICS = {
    "/sim/sensors/imu/data": Imu,
    "/imu/data": Imu,
    "/sim/truth_state": SimState,
    "/attitude/euler": Vector3Stamped,
}


def quat_to_euler(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def stamp_seconds(msg, received_ns: int) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
        return received_ns * 1e-9
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def read(path: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(path), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    rows: dict[str, list[tuple[float, np.ndarray]]] = {topic: [] for topic in TOPICS}
    while reader.has_next():
        topic, raw, received_ns = reader.read_next()
        msg_type = TOPICS.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(raw, msg_type)
        if topic.endswith("imu/data"):
            value = np.array([
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            ])
        elif topic == "/sim/truth_state":
            value = quat_to_euler(msg.pose.orientation)
        else:
            value = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        rows[topic].append((stamp_seconds(msg, received_ns), value))
    return {
        topic: (np.asarray([row[0] for row in values]), np.asarray([row[1] for row in values]))
        for topic, values in rows.items()
    }


def interpolate(t_source: np.ndarray, values: np.ndarray, t_target: np.ndarray) -> np.ndarray:
    return np.column_stack([
        np.interp(t_target, t_source, np.unwrap(values[:, axis])) for axis in range(values.shape[1])
    ])


def euler_to_quat(euler: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = euler
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    ])


def quat_to_euler_array(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    ])


def replay_estimator(
    imu: tuple[np.ndarray, np.ndarray],
    truth: tuple[np.ndarray, np.ndarray],
    start: float,
    end: float,
    c_product: bool,
    c_sequential_integrator: bool,
) -> np.ndarray:
    times, values = imu
    mask = (times >= start) & (times <= end)
    times = times[mask]
    values = values[mask]
    q = euler_to_quat(interpolate(truth[0], truth[1], np.array([times[0]]))[0])
    accel_lpf = values[0, :3].copy()
    gyro_lpf = values[0, 3:].copy()
    w1 = gyro_lpf.copy()
    w2 = gyro_lpf.copy()
    bias = np.zeros(3)
    estimates = []
    estimate_times = []
    previous = times[0]
    for timestamp, sample in zip(times[1:], values[1:]):
        dt = timestamp - previous
        previous = timestamp
        if dt <= 0 or dt > 0.02:
            continue
        accel_lpf = 0.5 * sample[:3] + 0.5 * accel_lpf
        gyro_lpf[:2] = 0.7 * sample[3:5] + 0.3 * gyro_lpf[:2]
        gyro_lpf[2] = 0.7 * sample[5] + 0.3 * gyro_lpf[2]
        norm_accel = np.linalg.norm(accel_lpf)
        w_err = np.zeros(3)
        kp = 0.0
        if 0.9 * 9.80665 < norm_accel < 1.1 * 9.80665:
            ax, ay, az = accel_lpf / norm_accel
            q_acc = np.array([1 - az, ay, -ax, 0.0])
            q_acc /= np.linalg.norm(q_acc)
            aw, ai, aj, ak = q_acc
            w, x, y, z = q
            if c_product:
                tw = aw * w - ai * x - aj * y - ak * z
                tx = aw * x + ai * w - aj * z + ak * y
                ty = aw * y + ai * z + aj * w - ak * x
            else:
                tw = aw * w - ai * x - aj * y - ak * z
                tx = aw * x + ai * w + aj * z - ak * y
                ty = aw * y - ai * z + aj * w + ak * x
            w_err = np.array([-2 * tw * tx, -2 * tw * ty, 0.0])
            kp = 0.5
        bias -= 0.01 * w_err * dt
        wbar = -w2 / 12 + w1 * (8 / 12) + gyro_lpf * (5 / 12)
        w2, w1 = w1, gyro_lpf.copy()
        p, qr, r = wbar - bias + kp * w_err
        norm_w = math.sqrt(p * p + qr * qr + r * r)
        if norm_w > 0:
            t1 = math.cos(norm_w * dt / 2)
            t2 = math.sin(norm_w * dt / 2) / norm_w
            if c_sequential_integrator:
                q[0] = t1 * q[0] + t2 * (-p * q[1] - qr * q[2] - r * q[3])
                q[1] = t1 * q[1] + t2 * (p * q[0] + r * q[2] - qr * q[3])
                q[2] = t1 * q[2] + t2 * (qr * q[0] - r * q[1] + p * q[3])
                q[3] = t1 * q[3] + t2 * (r * q[0] + qr * q[1] - p * q[2])
            else:
                old = q.copy()
                q = np.array([
                    t1 * old[0] + t2 * (-p * old[1] - qr * old[2] - r * old[3]),
                    t1 * old[1] + t2 * (p * old[0] + r * old[2] - qr * old[3]),
                    t1 * old[2] + t2 * (qr * old[0] - r * old[1] + p * old[3]),
                    t1 * old[3] + t2 * (r * old[0] + qr * old[1] - p * old[2]),
                ])
            q /= np.linalg.norm(q)
        estimates.append(quat_to_euler_array(q))
        estimate_times.append(timestamp)
    estimate_times = np.asarray(estimate_times)
    estimates = np.asarray(estimates)
    actual = interpolate(truth[0], truth[1], estimate_times)
    error = (estimates - actual + np.pi) % (2 * np.pi) - np.pi
    settled = estimate_times >= start + 5.0
    return np.sqrt(np.mean(error[settled] ** 2, axis=0))


def best_lag(
    reference: tuple[np.ndarray, np.ndarray],
    measured: tuple[np.ndarray, np.ndarray],
    start: float,
    end: float,
    max_lag: float,
) -> tuple[float, np.ndarray, np.ndarray]:
    ref_t, ref_v = reference
    meas_t, meas_v = measured
    mask = (meas_t >= start) & (meas_t <= end)
    sample_t = meas_t[mask][::10]
    sample_v = meas_v[mask][::10]
    best: tuple[float, np.ndarray, np.ndarray] | None = None
    for lag in np.arange(-max_lag, max_lag + 0.00025, 0.00025):
        predicted = interpolate(ref_t, ref_v, sample_t + lag)
        error = sample_v - predicted
        error[:, : min(3, error.shape[1])] = (
            error[:, : min(3, error.shape[1])] + np.pi
        ) % (2 * np.pi) - np.pi
        rms = np.sqrt(np.mean(error * error, axis=0))
        score = float(np.mean(rms[:2]))
        if best is None or score < float(np.mean(best[1][:2])):
            best = (float(lag), rms, np.mean(error, axis=0))
    assert best is not None
    return best


def analyze(label: str, path: Path) -> None:
    data = read(path)
    origin = min(series[0][0] for series in data.values() if series[0].size)
    data = {key: (value[0] - origin, value[1]) for key, value in data.items()}
    sim_imu = data["/sim/sensors/imu/data"]
    fw_imu = data["/imu/data"]
    truth = data["/sim/truth_state"]
    attitude = data["/attitude/euler"]

    imu_lag, imu_rms, imu_bias = best_lag(sim_imu, fw_imu, 5.0, 120.0, 0.03)
    att_lag, att_rms, att_bias = best_lag(truth, attitude, 10.0, 120.0, 0.10)
    att_mask = (attitude[0] >= 10.0) & (attitude[0] <= 45.0)
    att_sample = attitude[1][att_mask]
    truth_at_att = interpolate(truth[0], truth[1], attitude[0][att_mask])
    slopes = []
    correlations = []
    for axis in range(2):
        slopes.append(float(np.polyfit(truth_at_att[:, axis], att_sample[:, axis], 1)[0]))
        correlations.append(float(np.corrcoef(truth_at_att[:, axis], att_sample[:, axis])[0, 1]))
    cross_correlation = np.corrcoef(truth_at_att[:, :2].T, att_sample[:, :2].T)[:2, 2:]
    print(f"{label}: {path}")
    print(f"  counts sim_imu={len(sim_imu[0])} fw_imu={len(fw_imu[0])} "
          f"truth={len(truth[0])} attitude={len(attitude[0])}")
    print(f"  firmware IMU best header-time lag={imu_lag * 1e3:+.2f} ms")
    print("  firmware-minus-sim IMU RMS "
          f"accel={imu_rms[:3]} gyro={imu_rms[3:]}")
    print("  firmware-minus-sim IMU bias "
          f"accel={imu_bias[:3]} gyro={imu_bias[3:]}")
    print(f"  attitude best truth-time lag={att_lag * 1e3:+.2f} ms")
    print(f"  attitude-minus-truth RMS rpy={att_rms}")
    print(f"  attitude-minus-truth bias rpy={att_bias}")
    print(f"  first-leg estimate-vs-truth slope roll/pitch={slopes}")
    print(f"  first-leg estimate-vs-truth correlation roll/pitch={correlations}")
    print(f"  first-leg truth-axis x estimate-axis correlation=\n{cross_correlation}")
    for variant, c_product, c_integrator in (
        ("pre-fix Hamilton product + simultaneous integration", False, False),
        ("corrected turbomath product + simultaneous integration", True, False),
        ("pre-fix Hamilton product + C sequential integration", False, True),
        ("corrected turbomath product + C sequential integration", True, True),
    ):
        replay_rms = replay_estimator(fw_imu, truth, 10.0, 45.0, c_product, c_integrator)
        print(f"  replay {variant}: truth RMS rpy={replay_rms}")


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--veloxity-bag",
        type=Path,
        default=(
            root
            / "takeoff_logs"
            / "quadx_upstream_backend_compare_veloxity_accel_quat_fix_repeat"
        ),
    )
    parser.add_argument(
        "--c-bag", type=Path, default=root / "takeoff_logs/quadx_upstream_backend_compare_c"
    )
    args = parser.parse_args()
    analyze("Veloxity", args.veloxity_bag)
    analyze("C", args.c_bag)


if __name__ == "__main__":
    main()
