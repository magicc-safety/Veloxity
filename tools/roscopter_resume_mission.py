#!/usr/bin/env python3
"""Build and load a ROScopter resume mission from the current vehicle state."""

from __future__ import annotations

import argparse
import math
import tempfile
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rosflight_msgs.msg import SimState
from rosflight_msgs.srv import ParamFile
from roscopter_msgs.msg import State


def parse_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def parse_mission(path: Path) -> list[dict[str, Any]]:
    waypoints: list[dict[str, Any]] = []
    current: list[str] = []

    for raw_line in path.read_text().splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if raw_line.startswith("wp:"):
            if current:
                waypoints.append(yaml.safe_load("\n".join(current))["wp"])
            current = [raw_line]
        elif current:
            current.append(raw_line)

    if current:
        waypoints.append(yaml.safe_load("\n".join(current))["wp"])

    if not waypoints:
        raise ValueError(f"no wp blocks found in {path}")
    return [normalize_waypoint(wp) for wp in waypoints]


def normalize_waypoint(wp: dict[str, Any]) -> dict[str, Any]:
    return {
        "type": int(wp.get("type", 1)),
        "w": [float(value) for value in wp.get("w", [0.0, 0.0, 0.0])],
        "speed": float(wp.get("speed", 4.0)),
        "psi": float(wp.get("psi", 0.0)),
        "use_lla": parse_bool(wp.get("use_lla"), False),
        "hold_seconds": float(wp.get("hold_seconds", 0.0)),
        "hold_indefinitely": parse_bool(wp.get("hold_indefinitely"), False),
    }


def waypoint_yaml(wp: dict[str, Any]) -> str:
    return "\n".join(
        [
            "wp:",
            f"  type: {int(wp['type'])}",
            "  w: [" + ", ".join(f"{float(value):.6f}" for value in wp["w"]) + "]",
            f"  speed: {float(wp['speed']):.6f}",
            f"  psi: {float(wp['psi']):.6f}",
            f"  use_lla: {str(bool(wp['use_lla'])).lower()}",
            f"  hold_seconds: {float(wp['hold_seconds']):.6f}",
            f"  hold_indefinitely: {str(bool(wp['hold_indefinitely'])).lower()}",
        ]
    )


def write_mission(path: Path, waypoints: list[dict[str, Any]]) -> None:
    path.write_text("\n".join(waypoint_yaml(wp) for wp in waypoints) + "\n")


def choose_resume_index(waypoints: list[dict[str, Any]], current: tuple[float, float, float]) -> int:
    north, east, _down = current
    ned_waypoints = [(idx, wp) for idx, wp in enumerate(waypoints) if not wp["use_lla"]]
    if not ned_waypoints:
        return 0

    nearest_idx, _nearest_wp = min(
        ned_waypoints,
        key=lambda item: math.hypot(item[1]["w"][0] - north, item[1]["w"][1] - east),
    )
    return min(nearest_idx + 1, len(waypoints) - 1)


class ResumeMissionNode:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.node = rclpy.create_node("roscopter_resume_mission")
        self.state: State | SimState | None = None
        msg_type = SimState if args.state_type == "truth" else State
        self.sub = self.node.create_subscription(msg_type, args.state_topic, self.state_callback, 10)
        self.load_client = self.node.create_client(ParamFile, args.load_service)

    def state_callback(self, msg: State | SimState) -> None:
        self.state = msg

    def wait_for_state(self) -> State | SimState:
        deadline = time.monotonic() + self.args.timeout_s
        while rclpy.ok() and self.state is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.state is None:
            raise TimeoutError(f"timed out waiting for {self.args.state_topic}")
        return self.state

    def load_mission(self, mission_path: Path) -> tuple[bool, str]:
        if not self.load_client.wait_for_service(timeout_sec=self.args.timeout_s):
            raise TimeoutError(f"timed out waiting for {self.args.load_service}")

        request = ParamFile.Request()
        request.filename = str(mission_path)
        future = self.load_client.call_async(request)
        deadline = time.monotonic() + self.args.timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError(f"timed out calling {self.args.load_service}")
        response = future.result()
        return bool(response.success), str(getattr(response, "message", ""))

    def close(self) -> None:
        self.node.destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("mission", type=Path)
    parser.add_argument("--state-topic", default="/estimated_state")
    parser.add_argument(
        "--state-type",
        choices=["estimated", "truth"],
        default="estimated",
        help="Use roscopter_msgs/State or rosflight_msgs/SimState for current NED.",
    )
    parser.add_argument("--load-service", default="/path_planner/load_mission_from_file")
    parser.add_argument("--resume-index", type=int)
    parser.add_argument("--no-rejoin", action="store_true")
    parser.add_argument("--rejoin-speed", type=float, default=4.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.state_type == "truth" and args.state_topic == "/estimated_state":
        args.state_topic = "/sim/truth_state"
    waypoints = parse_mission(args.mission)

    rclpy.init()
    runner = ResumeMissionNode(args)
    try:
        state = runner.wait_for_state()
        if args.state_type == "truth":
            position = state.pose.position
            current = (float(position.x), float(position.y), float(position.z))
            current_psi = waypoints[0]["psi"]
        else:
            current = (float(state.p_n), float(state.p_e), float(state.p_d))
            current_psi = float(getattr(state, "psi", waypoints[0]["psi"]))
        resume_index = (
            args.resume_index
            if args.resume_index is not None
            else choose_resume_index(waypoints, current)
        )
        resume_index = max(0, min(resume_index, len(waypoints) - 1))

        resumed = waypoints[resume_index:]
        if not args.no_rejoin:
            target = resumed[0]
            rejoin = {
                "type": 1,
                "w": [current[0], current[1], current[2]],
                "speed": args.rejoin_speed,
                "psi": current_psi,
                "use_lla": False,
                "hold_seconds": 0.0,
                "hold_indefinitely": False,
            }
            resumed = [rejoin] + resumed

        output = args.output
        if output is None:
            handle = tempfile.NamedTemporaryFile(
                prefix="roscopter-resume-", suffix=".yaml", delete=False
            )
            output = Path(handle.name)
            handle.close()
        write_mission(output, resumed)

        success, message = runner.load_mission(output)
        print(f"current_ned: [{current[0]:.3f}, {current[1]:.3f}, {current[2]:.3f}]")
        print(f"resume_index: {resume_index}")
        print(f"mission_file: {output}")
        print(f"waypoints_loaded: {len(resumed)}")
        print(f"load_success: {success}")
        if message:
            print(f"load_message: {message}")
        if not success:
            raise SystemExit(1)
    finally:
        runner.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
