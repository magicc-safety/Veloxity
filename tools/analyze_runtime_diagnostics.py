#!/usr/bin/env python3
"""Analyze one or more JSON records from capture_runtime_diagnostics.py."""

from __future__ import annotations

import argparse
import csv
import json
import pathlib
import statistics
import sys

CORE_FIELDS = (
    "publish_hz",
    "published",
    "errors",
    "signal_overwrites",
    "queue_full_waits",
    "queue_wait_avg_us",
    "queue_wait_max_us",
    "queue_depth_max",
    "conversion_commands",
    "drdy_ready",
    "drdy_misses",
    "i2c_errors",
    "consumed",
    "consumed_hz",
    "consume_errors",
    "consume_age_avg_us",
    "consume_age_max_us",
    "processed_in",
    "processed_in_hz",
    "processed_out",
    "processed_out_hz",
    "unsent_overwrites",
    "telemetry_sent",
    "telemetry_sent_hz",
)


def load(path: pathlib.Path) -> dict:
    return json.loads(path.read_text())


def sensor_map(record: dict) -> dict[str, dict]:
    return {row["sensor"]: row for row in record["sensors"]}


def summary(path: pathlib.Path, record: dict) -> None:
    print(f"{path}: {record['duration_seconds']:.3f} s, {record['captured_at_utc']}")
    columns = ("sensor", "publish_hz", "published", "signal_overwrites", "queue_full_waits", "queue_depth_max", "consumed", "processed_out", "unsent_overwrites", "telemetry_sent")
    print(" ".join(f"{column:>18}" for column in columns))
    for row in record["sensors"]:
        values = []
        for column in columns:
            value = row.get(column, "-")
            values.append(f"{value:18.3f}" if isinstance(value, float) else f"{str(value):>18}")
        print(" ".join(values))
    mag = next((row for row in record["sensors"] if row["sensor"] == "MAG"), None)
    if mag is not None and "conversion_commands" in mag:
        print("\nIST8308 acquisition")
        print(
            f"  commands={mag['conversion_commands']} "
            f"drdy_ready={mag['drdy_ready']} "
            f"drdy_misses={mag['drdy_misses']} "
            f"i2c_errors={mag['i2c_errors']}"
        )


def stats(paths: list[pathlib.Path]) -> None:
    records = [load(path) for path in paths]
    sensors = sorted({row["sensor"] for record in records for row in record["sensors"]})
    print(f"Runs: {len(records)}")
    for sensor in sensors:
        print(f"\n{sensor}")
        rows = [sensor_map(record).get(sensor, {}) for record in records]
        for field in CORE_FIELDS:
            values = [float(row[field]) for row in rows if field in row]
            if not values:
                continue
            stdev = statistics.stdev(values) if len(values) > 1 else 0.0
            print(
                f"  {field:24} mean={statistics.mean(values):12.4f} "
                f"stdev={stdev:12.4f} min={min(values):12.4f} max={max(values):12.4f}"
            )


def compare(base_path: pathlib.Path, candidate_path: pathlib.Path) -> None:
    base = sensor_map(load(base_path))
    candidate = sensor_map(load(candidate_path))
    print(f"base:      {base_path}")
    print(f"candidate: {candidate_path}")
    for sensor in sorted(set(base) | set(candidate)):
        print(f"\n{sensor}")
        for field in CORE_FIELDS:
            if field not in base.get(sensor, {}) or field not in candidate.get(sensor, {}):
                continue
            old = float(base[sensor][field])
            new = float(candidate[sensor][field])
            percent = "n/a" if old == 0 else f"{100.0 * (new - old) / old:+.2f}%"
            print(f"  {field:24} {old:12.4f} -> {new:12.4f}  delta={new-old:+12.4f} ({percent})")


def export_csv(paths: list[pathlib.Path], output) -> None:
    fields = ("file", "captured_at_utc", "duration_seconds", "sensor", *CORE_FIELDS)
    writer = csv.DictWriter(output, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()
    for path in paths:
        record = load(path)
        for sensor in record["sensors"]:
            writer.writerow(
                {
                    "file": str(path),
                    "captured_at_utc": record["captured_at_utc"],
                    "duration_seconds": record["duration_seconds"],
                    **sensor,
                }
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    one = subparsers.add_parser("summary")
    one.add_argument("record", type=pathlib.Path)
    many = subparsers.add_parser("stats")
    many.add_argument("records", type=pathlib.Path, nargs="+")
    comparison = subparsers.add_parser("compare")
    comparison.add_argument("base", type=pathlib.Path)
    comparison.add_argument("candidate", type=pathlib.Path)
    export = subparsers.add_parser("export-csv")
    export.add_argument("records", type=pathlib.Path, nargs="+")
    export.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()

    if args.command == "summary":
        summary(args.record, load(args.record))
    elif args.command == "stats":
        stats(args.records)
    elif args.command == "compare":
        compare(args.base, args.candidate)
    elif args.command == "export-csv":
        if args.output:
            with args.output.open("w", newline="") as output:
                export_csv(args.records, output)
        else:
            export_csv(args.records, sys.stdout)


if __name__ == "__main__":
    main()
