#!/usr/bin/env python3
"""Replay a ros2 topic echo YAML stream as rosflight_msgs/msg/Command."""

from __future__ import annotations

import argparse
import re
import time
from pathlib import Path

import rclpy
import yaml
from rosflight_msgs.msg import Command


def load_messages(path: Path) -> list[dict]:
    text = path.read_text()
    first_doc = text.find("---")
    if first_doc > 0:
        text = text[first_doc:]
    text = text.replace("\t", "  ")
    text = re.sub(r"(count:\s*\d+)---", r"\1\n---", text)
    lines = []
    for line in text.splitlines():
        if line.startswith("!") or line.startswith("A message"):
            continue
        if line.strip().startswith("total count"):
            continue
        lines.append(line)

    docs = []
    for doc in yaml.safe_load_all("\n".join(lines)):
        if isinstance(doc, dict) and isinstance(doc.get("u"), list):
            docs.append(doc)
    return docs


def stamp_seconds(doc: dict) -> float | None:
    stamp = (doc.get("header") or {}).get("stamp") or {}
    sec = stamp.get("sec")
    nanosec = stamp.get("nanosec")
    if isinstance(sec, int) and isinstance(nanosec, int):
        return sec + nanosec * 1e-9
    return None


def to_command(doc: dict, now) -> Command:
    msg = Command()
    msg.header.stamp = now
    msg.mode = int(doc.get("mode", Command.MODE_PASS_THROUGH))
    msg.ignore = int(doc.get("ignore", Command.IGNORE_NONE))
    values = [float(value) for value in doc["u"][:10]]
    msg.u = values + [0.0] * (10 - len(values))
    return msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("yaml", type=Path)
    parser.add_argument("--topic", default="/command")
    parser.add_argument("--skip", type=int, default=0)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=0.0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--warmup-s", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    docs = load_messages(args.yaml)
    if args.skip:
        docs = docs[args.skip :]
    if args.limit:
        docs = docs[: args.limit]
    if not docs:
        raise SystemExit("no command messages to replay")

    rclpy.init()
    node = rclpy.create_node("rosflight_command_yaml_replay")
    pub = node.create_publisher(Command, args.topic, 10)

    time.sleep(args.warmup_s)
    start_wall = time.monotonic()
    first_stamp = stamp_seconds(docs[0])
    fixed_period = 1.0 / args.rate_hz if args.rate_hz > 0.0 else None

    for index, doc in enumerate(docs):
        if fixed_period is not None:
            target = start_wall + index * fixed_period
        else:
            current_stamp = stamp_seconds(doc)
            if first_stamp is None or current_stamp is None:
                target = start_wall
            else:
                target = start_wall + (current_stamp - first_stamp) / args.speed
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        pub.publish(to_command(doc, node.get_clock().now().to_msg()))
        rclpy.spin_once(node, timeout_sec=0.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
