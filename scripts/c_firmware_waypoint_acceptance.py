#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

import rclpy
from rosflight_msgs.msg import Command, SimState
from rosflight_msgs.srv import ParamFile
from roscopter_msgs.msg import ControllerCommand, TrajectoryCommand, Waypoint
from visualization_msgs.msg import Marker

from sil_test_lib import (
    LaunchConfig,
    SilProbe,
    add_common_args,
    distance_to,
    launch_stack,
    require_ros_environment,
    start_bag,
    stop_processes,
)


TUTORIAL_WAYPOINTS = [
    (0.0, 0.0, -10.0),
    (20.0, 0.0, -10.0),
    (20.0, -20.0, -20.0),
    (0.0, -20.0, -20.0),
    (0.0, 0.0, -40.0),
]


class WaypointProbe(SilProbe):
    def __init__(self) -> None:
        self.truth_samples: list[tuple[float, int, int, float, float, float, float, float, float]] = []
        super().__init__("c_firmware_waypoint_acceptance")
        self.waypoints_seen = 0
        self.trajectory_seen = 0
        self.high_level_seen = 0
        self.rviz_markers_seen = 0
        self.create_subscription(Waypoint, "/waypoints", self._waypoint_cb, 10)
        self.create_subscription(TrajectoryCommand, "/trajectory_command", self._trajectory_cb, 10)
        self.create_subscription(ControllerCommand, "/high_level_command", self._high_level_cb, 10)
        self.create_subscription(Marker, "/rviz/waypoint", self._marker_cb, 10)
        self.load_mission = self.create_client(ParamFile, "/path_planner/load_mission_from_file")

    def _truth_cb(self, msg: SimState) -> None:
        super()._truth_cb(msg)
        self.truth_samples.append((
            time.time(),
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
        ))

    def _waypoint_cb(self, msg: Waypoint) -> None:
        if not msg.clear_wp_list:
            self.waypoints_seen += 1

    def _trajectory_cb(self, msg: TrajectoryCommand) -> None:
        self.trajectory_seen += 1

    def _high_level_cb(self, msg: ControllerCommand) -> None:
        self.high_level_seen += 1

    def _marker_cb(self, msg: Marker) -> None:
        if msg.ns in ("wp", "text") and msg.action != Marker.DELETEALL:
            self.rviz_markers_seen += 1

    def call_load_mission(self, filename: str) -> None:
        if not self.load_mission.wait_for_service(timeout_sec=45.0):
            raise RuntimeError("/path_planner/load_mission_from_file service is not available")
        req = ParamFile.Request()
        req.filename = filename
        future = self.load_mission.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=45.0)
        result = future.result() if future.done() else None
        if result is None or not result.success:
            raise RuntimeError(f"failed to load mission from {filename}")

    def wait_for_visit(self, target: tuple[float, float, float], tolerance: float, timeout_s: float) -> SimState:
        deadline = time.time() + timeout_s
        closest = float("inf")
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.truth is None:
                continue
            closest = min(closest, distance_to(self.truth, target))
            if closest <= tolerance:
                return self.truth
        raise RuntimeError(f"timed out before reaching {target}; closest={closest:.2f}m")

    def wait_command_chain(self, timeout_s: float) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.trajectory_seen > 0 and self.high_level_seen > 0 and self.last_command is not None:
                return
        raise RuntimeError(
            "ROScopter command chain did not become active after mission load: "
            f"trajectory={self.trajectory_seen}, high_level={self.high_level_seen}, "
            f"command={self.last_command is not None}"
        )

    def write_outputs(self, csv_filename: str, png_filename: str, visited: list[tuple[float, float, float]]) -> None:
        self.write_csv(csv_filename, visited)
        self.write_png(png_filename, visited)

    def write_csv(self, filename: str, visited: list[tuple[float, float, float]]) -> None:
        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "wall_time_s",
                "ros_stamp_sec",
                "ros_stamp_nanosec",
                "north_m",
                "east_m",
                "down_m",
                "velocity_n_mps",
                "velocity_e_mps",
                "velocity_d_mps",
            ])
            writer.writerows(self.truth_samples)

        visits_path = path.with_name(path.stem + "-waypoint-visits.csv")
        with visits_path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["waypoint_index", "target_n_m", "target_e_m", "target_d_m", "reached_n_m", "reached_e_m", "reached_d_m"])
            for index, (target, reached) in enumerate(zip(TUTORIAL_WAYPOINTS, visited), start=1):
                writer.writerow([index, *target, *reached])
        print(f"WROTE path_csv={path}")
        print(f"WROTE waypoint_visits_csv={visits_path}")

    def write_png(self, filename: str, visited: list[tuple[float, float, float]]) -> None:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            raise RuntimeError(f"matplotlib is required to write PNG path plot: {exc}") from exc

        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        north = [sample[3] for sample in self.truth_samples]
        east = [sample[4] for sample in self.truth_samples]
        target_n = [wp[0] for wp in TUTORIAL_WAYPOINTS]
        target_e = [wp[1] for wp in TUTORIAL_WAYPOINTS]
        reached_n = [point[0] for point in visited]
        reached_e = [point[1] for point in visited]

        fig, ax = plt.subplots(figsize=(8, 8), dpi=140)
        ax.plot(east, north, linewidth=1.5, label="truth path")
        ax.scatter(target_e, target_n, marker="x", s=70, label="target waypoints")
        ax.scatter(reached_e, reached_n, marker="o", s=28, label="accepted samples")
        for index, (n, e, _d) in enumerate(TUTORIAL_WAYPOINTS, start=1):
            ax.annotate(str(index), (e, n), xytext=(5, 5), textcoords="offset points")
        ax.set_title("C Firmware ROScopter Tutorial Mission")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.grid(True, linewidth=0.4, alpha=0.5)
        ax.axis("equal")
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(path)
        plt.close(fig)
        print(f"WROTE path_png={path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the ROScopter tutorial mission against C firmware.")
    add_common_args(parser)
    parser.add_argument(
        "--mission-file",
        default="/home/skink/projects/rosflight_setup/workspace/install/roscopter/share/roscopter/params/multirotor_mission.yaml",
    )
    parser.add_argument("--waypoint-tolerance", type=float, default=4.0)
    parser.add_argument("--per-waypoint-timeout", type=float, default=70.0)
    parser.add_argument("--no-rosbag", action="store_true")
    parser.add_argument(
        "--csv-output",
        default="target/waypoint_paths/c-firmware-waypoints.csv",
        help="CSV file for sampled /sim/truth_state path data.",
    )
    parser.add_argument(
        "--png-output",
        default="target/waypoint_paths/c-firmware-waypoints.png",
        help="PNG plot for sampled /sim/truth_state path data.",
    )
    args = parser.parse_args()
    require_ros_environment()

    processes = launch_stack(LaunchConfig(
        firmware=args.firmware,
        use_builtin_rc=True,
        use_rviz=not args.no_rviz,
        use_roscopter=True,
    ))
    bag = None
    rclpy.init()
    node = WaypointProbe()
    try:
        node.wait_ready()
        node.initialize_firmware()
        node.call_load_mission(args.mission_file)
        node.call_trigger(node.toggle_arm, "/toggle_arm")
        node.wait_armed()
        node.call_trigger(node.toggle_override, "/toggle_override")
        node.wait_command_chain(timeout_s=20.0)
        node.wait_offboard()
        if not args.no_rosbag:
            bag = start_bag("c-firmware-waypoints", [
                "/sim/truth_state",
                "/status",
                "/command",
                "/trajectory_command",
                "/high_level_command",
                "/waypoints",
            ])
        visited = []
        for waypoint in TUTORIAL_WAYPOINTS:
            sample = node.wait_for_visit(waypoint, args.waypoint_tolerance, args.per_waypoint_timeout)
            pos = sample.pose.position
            visited.append((pos.x, pos.y, pos.z))
            print(f"PASS waypoint {waypoint}: reached ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        if node.trajectory_seen == 0 or node.high_level_seen == 0 or node.last_command is None:
            raise RuntimeError(
                "mission reached positions but ROScopter command chain was incomplete: "
                f"trajectory={node.trajectory_seen}, high_level={node.high_level_seen}, command={node.last_command is not None}"
            )
        print(
            "PASS waypoint mission: "
            f"waypoints_seen={node.waypoints_seen}, rviz_markers={node.rviz_markers_seen}, "
            f"trajectory={node.trajectory_seen}, command_mode={node.last_command.mode}"
        )
        node.write_outputs(args.csv_output, args.png_output, visited)
        return 0
    except Exception as exc:
        print(f"FAIL waypoint mission: {exc}", file=sys.stderr)
        return 1
    finally:
        if bag is not None:
            stop_processes([bag])
        node.destroy_node()
        rclpy.shutdown()
        if not args.keep_running:
            stop_processes(processes)


if __name__ == "__main__":
    raise SystemExit(main())
