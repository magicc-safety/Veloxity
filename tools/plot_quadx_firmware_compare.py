#!/usr/bin/env python3
"""Plot Rust vs C quad-X waypoint comparison bags."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import RangeSlider
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from roscopter_msgs.msg import State, TrajectoryCommand
from rosflight_msgs.msg import Command, PwmOutput, SimState


TOPIC_TYPES = {
    "/sim/truth_state": SimState,
    "/estimated_state": State,
    "/trajectory_command": TrajectoryCommand,
    "/command": Command,
    "/sim/pwm_output": PwmOutput,
}


def stamp_s(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def unwrap_rad(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return values
    return np.unwrap(values)


def wrap_pi(values: np.ndarray) -> np.ndarray:
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def quat_to_euler(q) -> tuple[float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def empty_series() -> dict[str, list]:
    return {
        "truth_t": [],
        "truth_pos": [],
        "truth_vel": [],
        "truth_euler": [],
        "truth_rates": [],
        "truth_accel": [],
        "est_t": [],
        "est_pos": [],
        "est_vel": [],
        "est_euler": [],
        "est_rates": [],
        "traj_t": [],
        "traj_pos": [],
        "traj_vel": [],
        "traj_accel": [],
        "traj_psi": [],
        "traj_psi_dot": [],
        "cmd_t": [],
        "cmd_u": [],
        "pwm_t": [],
        "pwm": [],
    }


def read_bag(path: Path) -> dict[str, np.ndarray]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(path), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    data = empty_series()
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        msg_type = TOPIC_TYPES.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(raw, msg_type)
        t = stamp_s(msg)

        if topic == "/sim/truth_state":
            r, p, y = quat_to_euler(msg.pose.orientation)
            data["truth_t"].append(t)
            data["truth_pos"].append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            data["truth_vel"].append([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            data["truth_euler"].append([r, p, y])
            data["truth_rates"].append([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
            data["truth_accel"].append([msg.acceleration.linear.x, msg.acceleration.linear.y, msg.acceleration.linear.z])
        elif topic == "/estimated_state":
            data["est_t"].append(t)
            data["est_pos"].append([msg.p_n, msg.p_e, msg.p_d])
            data["est_vel"].append([msg.v_x, msg.v_y, msg.v_z])
            data["est_euler"].append([msg.phi, msg.theta, msg.psi])
            data["est_rates"].append([msg.p, msg.q, msg.r])
        elif topic == "/trajectory_command":
            data["traj_t"].append(t)
            data["traj_pos"].append(list(msg.position))
            data["traj_vel"].append(list(msg.velocity))
            data["traj_accel"].append(list(msg.acceleration))
            data["traj_psi"].append(msg.psi)
            data["traj_psi_dot"].append(msg.psi_dot)
        elif topic == "/command":
            data["cmd_t"].append(t)
            data["cmd_u"].append(list(msg.u))
        elif topic == "/sim/pwm_output":
            data["pwm_t"].append(t)
            data["pwm"].append(list(msg.values))

    out: dict[str, np.ndarray] = {}
    for key, value in data.items():
        out[key] = np.asarray(value, dtype=float)

    t0 = min(
        arr[0]
        for key, arr in out.items()
        if key.endswith("_t") and arr.size > 0
    )
    for key in list(out):
        if key.endswith("_t") and out[key].size > 0:
            out[key] = out[key] - t0

    for key in ("truth_euler", "est_euler"):
        if out[key].size:
            out[key][:, 2] = unwrap_rad(out[key][:, 2])
    if out["traj_psi"].size:
        out["traj_psi"] = unwrap_rad(out["traj_psi"])
    return out


def interp_matrix(src_t: np.ndarray, src_y: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    if src_t.size == 0 or dst_t.size == 0:
        return np.empty((0, 0))
    if src_y.ndim == 1:
        return np.interp(dst_t, src_t, src_y)
    return np.column_stack([np.interp(dst_t, src_t, src_y[:, i]) for i in range(src_y.shape[1])])


def common_time(*arrays: np.ndarray, count: int = 2000) -> np.ndarray:
    starts = [a[0] for a in arrays if a.size]
    ends = [a[-1] for a in arrays if a.size]
    if not starts or not ends:
        return np.array([])
    start = max(starts)
    end = min(ends)
    if end <= start:
        return np.array([])
    return np.linspace(start, end, count)


def set_3d_equalish(ax, xs: list[np.ndarray], ys: list[np.ndarray], zs: list[np.ndarray]) -> None:
    vals = [arr for arr in xs + ys + zs if arr.size]
    if not vals:
        return
    x = np.concatenate([arr for arr in xs if arr.size])
    y = np.concatenate([arr for arr in ys if arr.size])
    z = np.concatenate([arr for arr in zs if arr.size])
    max_range = max(np.ptp(x), np.ptp(y), np.ptp(z), 1.0)
    ax.set_xlim(np.mean(x) - max_range / 2.0, np.mean(x) + max_range / 2.0)
    ax.set_ylim(np.mean(y) - max_range / 2.0, np.mean(y) + max_range / 2.0)
    ax.set_zlim(np.mean(z) - max_range / 2.0, np.mean(z) + max_range / 2.0)


def init_time_slider(fig, axes, t_min: float, t_max: float, callback):
    slider_ax = fig.add_axes([0.18, 0.025, 0.68, 0.025])
    slider = RangeSlider(
        slider_ax,
        "time [s]",
        t_min,
        t_max,
        valinit=(t_min, t_max),
        valstep=(t_max - t_min) / 1000.0 if t_max > t_min else None,
    )

    def on_change(window):
        start, end = window
        for ax in axes:
            ax.set_xlim(start, end)
        callback(start, end)
        fig.canvas.draw_idle()

    slider.on_changed(on_change)
    return slider


def time_bounds(*bags: dict[str, np.ndarray]) -> tuple[float, float]:
    starts = []
    ends = []
    for bag in bags:
        for key, value in bag.items():
            if key.endswith("_t") and value.size:
                starts.append(float(value[0]))
                ends.append(float(value[-1]))
    return min(starts), max(ends)


STATE_GRID = [
    ("p_N", "m", "pos", 0, False),
    ("p_E", "m", "pos", 1, False),
    ("p_D", "m", "pos", 2, False),
    ("v_N", "m/s", "vel", 0, False),
    ("v_E", "m/s", "vel", 1, False),
    ("v_D", "m/s", "vel", 2, False),
    ("roll", "deg", "euler", 0, True),
    ("pitch", "deg", "euler", 1, True),
    ("yaw", "deg", "euler", 2, True),
    ("p", "deg/s", "rates", 0, True),
    ("q", "deg/s", "rates", 1, True),
    ("r", "deg/s", "rates", 2, True),
]


def plot_state_grid(
    title: str,
    t: np.ndarray,
    values: dict[str, np.ndarray],
    output: Path | None,
    color: str,
):
    fig, axes = plt.subplots(3, 4, figsize=(22, 12), sharex=True)
    fig.subplots_adjust(left=0.05, right=0.985, top=0.91, bottom=0.08, wspace=0.28, hspace=0.42)
    flat_axes = list(axes.ravel())

    for ax, (name, unit, _group, _idx, _angle) in zip(flat_axes, STATE_GRID):
        ax.plot(t, values[name], color=color, linewidth=1.3)
        ax.axhline(0.0, color="0.2", linewidth=0.8, alpha=0.45)
        ax.set_title(name)
        ax.set_ylabel(unit)
        ax.grid(True, alpha=0.25)
    for ax in flat_axes[-4:]:
        ax.set_xlabel("time [s]")

    fig.suptitle(title)
    if t.size:
        slider = init_time_slider(fig, flat_axes, float(t[0]), float(t[-1]), lambda _start, _end: None)
        fig._time_slider = slider
    if output is not None:
        fig.savefig(output, dpi=180)
        print(f"Wrote {output}")
    return fig


def plot_estimate_delta(rust: dict[str, np.ndarray], c: dict[str, np.ndarray], output: Path | None):
    t = common_time(rust["est_t"], c["est_t"])
    values: dict[str, np.ndarray] = {}
    for name, _unit, group, idx, angle in STATE_GRID:
        rust_values = interp_matrix(rust["est_t"], rust[f"est_{group}"], t)[:, idx]
        c_values = interp_matrix(c["est_t"], c[f"est_{group}"], t)[:, idx]
        delta = c_values - rust_values
        if angle:
            delta = wrap_pi(delta)
            delta = np.rad2deg(delta)
        values[name] = delta
    return plot_state_grid("Estimated state difference: C - Rust", t, values, output, "tab:purple")


def plot_truth_minus_rust(rust: dict[str, np.ndarray], output: Path | None):
    t = common_time(rust["truth_t"], rust["est_t"])
    values: dict[str, np.ndarray] = {}
    for name, _unit, group, idx, angle in STATE_GRID:
        truth_values = interp_matrix(rust["truth_t"], rust[f"truth_{group}"], t)[:, idx]
        rust_values = interp_matrix(rust["est_t"], rust[f"est_{group}"], t)[:, idx]
        delta = truth_values - rust_values
        if angle:
            delta = wrap_pi(delta)
            delta = np.rad2deg(delta)
        values[name] = delta
    return plot_state_grid("Rust estimator error: truth - Rust estimate", t, values, output, "tab:orange")


def plot_overview(rust: dict[str, np.ndarray], c: dict[str, np.ndarray], output: Path | None):
    fig = plt.figure(figsize=(23, 13))
    fig.subplots_adjust(left=0.04, right=0.98, top=0.92, bottom=0.08, wspace=0.30, hspace=0.42)
    grid = fig.add_gridspec(3, 3, width_ratios=[1.55, 1.0, 1.0])
    ax3d = fig.add_subplot(grid[:, 0], projection="3d")
    axes = [
        fig.add_subplot(grid[0, 1]),
        fig.add_subplot(grid[0, 2]),
        fig.add_subplot(grid[1, 1]),
        fig.add_subplot(grid[1, 2]),
        fig.add_subplot(grid[2, 1]),
        fig.add_subplot(grid[2, 2]),
    ]

    colors = {"rust": "tab:orange", "c": "tab:blue"}
    bags = {"rust": rust, "c": c}
    traj_handles = []
    for name, bag in bags.items():
        pos = bag["truth_pos"]
        traj = bag["traj_pos"]
        if pos.size:
            handle, = ax3d.plot(pos[:, 0], pos[:, 1], -pos[:, 2], color=colors[name], label=f"{name} truth", linewidth=2)
            traj_handles.append((handle, bag["truth_t"], pos, True))
        if traj.size:
            handle, = ax3d.plot(traj[:, 0], traj[:, 1], -traj[:, 2], color=colors[name], linestyle="--", alpha=0.55, label=f"{name} carrot")
            traj_handles.append((handle, bag["traj_t"], traj, True))

    ax3d.set_title("3D trajectory")
    ax3d.set_xlabel("North [m]")
    ax3d.set_ylabel("East [m]")
    ax3d.set_zlabel("Up [-Down, m]")
    ax3d.legend(loc="upper left")

    for name, bag in bags.items():
        if bag["pwm"].size:
            axes[0].plot(bag["pwm_t"], bag["pwm"][:, :4], color=colors[name], alpha=0.35)
            axes[0].plot([], [], color=colors[name], label=name)
        if bag["est_euler"].size:
            axes[1].plot(bag["est_t"], np.rad2deg(bag["est_euler"][:, 0]), color=colors[name], label=name)
            axes[2].plot(bag["est_t"], np.rad2deg(bag["est_euler"][:, 1]), color=colors[name], label=name)
            axes[3].plot(bag["est_t"], np.rad2deg(bag["est_euler"][:, 2]), color=colors[name], label=name)
        if bag["truth_accel"].size:
            axes[4].plot(bag["truth_t"], bag["truth_accel"][:, 0], color=colors[name], linestyle="-", label=f"{name} ax")
            axes[4].plot(bag["truth_t"], bag["truth_accel"][:, 1], color=colors[name], linestyle="--", label=f"{name} ay")
            axes[4].plot(bag["truth_t"], bag["truth_accel"][:, 2], color=colors[name], linestyle=":", label=f"{name} az")
        if bag["truth_rates"].size:
            axes[5].plot(bag["truth_t"], np.rad2deg(bag["truth_rates"][:, 0]), color=colors[name], linestyle="-", label=f"{name} p")
            axes[5].plot(bag["truth_t"], np.rad2deg(bag["truth_rates"][:, 1]), color=colors[name], linestyle="--", label=f"{name} q")
            axes[5].plot(bag["truth_t"], np.rad2deg(bag["truth_rates"][:, 2]), color=colors[name], linestyle=":", label=f"{name} r")

    axes[0].set_title("PWM output, motor channels 0-3")
    axes[0].set_ylabel("PWM [us]")
    axes[1].set_title("Estimated roll")
    axes[1].set_ylabel("deg")
    axes[2].set_title("Estimated pitch")
    axes[2].set_ylabel("deg")
    axes[3].set_title("Estimated yaw")
    axes[3].set_ylabel("deg")
    axes[4].set_title("Simulator body acceleration")
    axes[4].set_ylabel("m/s^2")
    axes[5].set_title("Simulator body gyro/angular rate")
    axes[5].set_ylabel("deg/s")
    axes[4].set_xlabel("time [s]")
    axes[5].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=2, fontsize="small")

    fig.suptitle("Quad-X waypoint firmware comparison")
    t_min, t_max = time_bounds(rust, c)

    def update_3d(start: float, end: float) -> None:
        xs: list[np.ndarray] = []
        ys: list[np.ndarray] = []
        zs: list[np.ndarray] = []
        for handle, t, pos, _ in traj_handles:
            mask = (t >= start) & (t <= end)
            window = pos[mask]
            if window.size:
                x = window[:, 0]
                y = window[:, 1]
                z = -window[:, 2]
            else:
                x = y = z = np.array([])
            handle.set_data(x, y)
            handle.set_3d_properties(z)
            xs.append(x)
            ys.append(y)
            zs.append(z)
        set_3d_equalish(ax3d, xs, ys, zs)

    slider = init_time_slider(fig, axes, t_min, t_max, update_3d)
    fig._time_slider = slider
    if output is not None:
        fig.savefig(output, dpi=180)
        print(f"Wrote {output}")
    return fig


def plot_errors(rust: dict[str, np.ndarray], c: dict[str, np.ndarray], output: Path | None):
    fig, axes = plt.subplots(5, 1, figsize=(15, 13), sharex=True)
    fig.subplots_adjust(bottom=0.08)
    colors = {"rust": "tab:orange", "c": "tab:blue"}
    bags = {"rust": rust, "c": c}

    for name, bag in bags.items():
        t = common_time(bag["truth_t"], bag["est_t"], bag["traj_t"])
        if t.size == 0:
            continue
        truth_pos = interp_matrix(bag["truth_t"], bag["truth_pos"], t)
        truth_vel = interp_matrix(bag["truth_t"], bag["truth_vel"], t)
        truth_rates = interp_matrix(bag["truth_t"], bag["truth_rates"], t)
        truth_acc = interp_matrix(bag["truth_t"], bag["truth_accel"], t)
        est_euler = interp_matrix(bag["est_t"], bag["est_euler"], t)
        traj_pos = interp_matrix(bag["traj_t"], bag["traj_pos"], t)
        traj_vel = interp_matrix(bag["traj_t"], bag["traj_vel"], t)
        traj_acc = interp_matrix(bag["traj_t"], bag["traj_accel"], t)
        traj_psi = interp_matrix(bag["traj_t"], bag["traj_psi"], t)
        traj_psi_dot = interp_matrix(bag["traj_t"], bag["traj_psi_dot"], t)

        pos_err = truth_pos - traj_pos
        vel_err = truth_vel - traj_vel
        acc_err = truth_acc - traj_acc
        yaw_err = wrap_pi(est_euler[:, 2] - traj_psi)
        yaw_rate_err = truth_rates[:, 2] - traj_psi_dot

        axes[0].plot(t, pos_err[:, 0], color=colors[name], linestyle="-", label=f"{name} N")
        axes[0].plot(t, pos_err[:, 1], color=colors[name], linestyle="--", label=f"{name} E")
        axes[0].plot(t, pos_err[:, 2], color=colors[name], linestyle=":", label=f"{name} D")

        axes[1].plot(t, vel_err[:, 0], color=colors[name], linestyle="-", label=f"{name} vN")
        axes[1].plot(t, vel_err[:, 1], color=colors[name], linestyle="--", label=f"{name} vE")
        axes[1].plot(t, vel_err[:, 2], color=colors[name], linestyle=":", label=f"{name} vD")

        axes[2].plot(t, acc_err[:, 0], color=colors[name], linestyle="-", label=f"{name} aX")
        axes[2].plot(t, acc_err[:, 1], color=colors[name], linestyle="--", label=f"{name} aY")
        axes[2].plot(t, acc_err[:, 2], color=colors[name], linestyle=":", label=f"{name} aZ")

        axes[3].plot(t, np.rad2deg(yaw_err), color=colors[name], label=name)
        axes[4].plot(t, np.rad2deg(yaw_rate_err), color=colors[name], label=name)

    titles = [
        "Position error: truth - moving carrot",
        "Velocity error: truth - moving carrot",
        "Acceleration error: truth body accel - trajectory accel",
        "Heading error: estimated yaw - commanded yaw",
        "Yaw-rate error: truth r - commanded psi_dot",
    ]
    ylabels = ["m", "m/s", "m/s^2", "deg", "deg/s"]
    for ax, title, ylabel in zip(axes, titles, ylabels):
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=3, fontsize="small")
    axes[-1].set_xlabel("time [s]")

    fig.suptitle("Drone vs moving carrot errors")
    t_min, t_max = time_bounds(rust, c)
    slider = init_time_slider(fig, axes, t_min, t_max, lambda _start, _end: None)
    fig._time_slider = slider
    if output is not None:
        fig.savefig(output, dpi=180)
        print(f"Wrote {output}")
    return fig


def main() -> None:
    home = Path.home()
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-bag", type=Path, default=home / "quadx_waypoint_angle_mode_rust_finalcal_20260709")
    parser.add_argument("--c-bag", type=Path, default=home / "quadx_waypoint_angle_mode_c_finalcal_20260709")
    parser.add_argument("--save", action="store_true", help="Save PNGs in addition to opening interactive windows.")
    parser.add_argument("--overview", type=Path, default=home / "quadx_firmware_compare_overview.png")
    parser.add_argument("--errors", type=Path, default=home / "quadx_firmware_compare_errors.png")
    parser.add_argument("--estimate-delta", type=Path, default=home / "quadx_firmware_compare_estimate_delta.png")
    parser.add_argument("--truth-rust", type=Path, default=home / "quadx_firmware_compare_truth_minus_rust.png")
    parser.add_argument("--no-show", action="store_true", help="Do not open interactive windows. Useful with --save.")
    args = parser.parse_args()

    rust = read_bag(args.rust_bag)
    c = read_bag(args.c_bag)
    plot_overview(rust, c, args.overview if args.save else None)
    plot_errors(rust, c, args.errors if args.save else None)
    plot_estimate_delta(rust, c, args.estimate_delta if args.save else None)
    plot_truth_minus_rust(rust, args.truth_rust if args.save else None)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
