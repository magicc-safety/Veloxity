#!/usr/bin/env python3
"""Analyze Saleae digital CSV exports from the Pico scope-timing build."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    k = (len(ordered) - 1) * q / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - k) + ordered[hi] * (k - lo)


def summarize(name: str, values: list[float], budgets_us: list[float]) -> None:
    if not values:
        print(f"{name}: n=0")
        return

    print(
        f"{name}: n={len(values)} "
        f"mean={statistics.fmean(values) * 1e6:.3f}us "
        f"min={min(values) * 1e6:.3f}us "
        f"p50={percentile(values, 50) * 1e6:.3f}us "
        f"p90={percentile(values, 90) * 1e6:.3f}us "
        f"p95={percentile(values, 95) * 1e6:.3f}us "
        f"p99={percentile(values, 99) * 1e6:.3f}us "
        f"max={max(values) * 1e6:.3f}us"
    )
    for budget_us in budgets_us:
        budget = budget_us * 1e-6
        over = sum(value > budget for value in values)
        worst_margin_us = (budget - max(values)) * 1e6
        p99_margin_us = (budget - percentile(values, 99)) * 1e6
        print(
            f"  budget {budget_us:.3f}us: over={over}/{len(values)} "
            f"({over / len(values) * 100:.2f}%), "
            f"p99_margin={p99_margin_us:.3f}us, worst_margin={worst_margin_us:.3f}us"
        )


def segments(rows: list[tuple[float, list[int]]], channel_index: int) -> list[tuple[int, float, float]]:
    current = rows[0][1][channel_index]
    start = rows[0][0]
    out: list[tuple[int, float, float]] = []
    for timestamp, values in rows[1:]:
        value = values[channel_index]
        if value != current:
            out.append((current, start, timestamp))
            current = value
            start = timestamp
    out.append((current, start, rows[-1][0]))
    return out


def overlaps(interval: tuple[float, float], candidates: list[tuple[float, float]]) -> bool:
    start, end = interval
    return any(candidate_start < end and candidate_end > start for candidate_start, candidate_end in candidates)


def load_rows(path: Path) -> tuple[list[str], list[tuple[float, list[int]]]]:
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        channel_names = header[1:]
        rows = [(float(row[0]), [int(value) for value in row[1:]]) for row in reader]
    if not rows:
        raise SystemExit(f"{path} has no samples")
    return channel_names, rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--whole-channel", type=int, default=1, help="Saleae channel index for GP18")
    parser.add_argument(
        "--whole-mode",
        choices=("toggle", "pulse"),
        default="toggle",
        help="GP18 mode: pass-boundary toggle for new captures, high pulse for older captures",
    )
    parser.add_argument("--control-channel", type=int, default=0, help="Saleae channel index for GP19")
    parser.add_argument(
        "--non-control-channel",
        type=int,
        default=None,
        help="Optional Saleae channel index for GP22",
    )
    parser.add_argument(
        "--budget-us",
        type=float,
        action="append",
        default=[600.0, 312.5],
        help="Loop budget in microseconds; repeat for multiple budgets",
    )
    args = parser.parse_args()

    channel_names, rows = load_rows(args.csv)
    print(f"file={args.csv} rows={len(rows)} duration={rows[-1][0] - rows[0][0]:.6f}s")
    print(f"channels={', '.join(channel_names)}")

    whole_segments = segments(rows, args.whole_channel)
    if args.whole_mode == "toggle":
        whole_passes = [(start, end) for _, start, end in whole_segments if end > start]
    else:
        whole_passes = [(start, end) for value, start, end in whole_segments if value == 1]
    control_high = [(start, end) for value, start, end in segments(rows, args.control_channel) if value == 1]
    non_control_high: list[tuple[float, float]] = []
    if args.non_control_channel is not None:
        if args.non_control_channel >= len(channel_names):
            raise SystemExit(
                f"--non-control-channel {args.non_control_channel} is outside CSV channel range"
            )
        non_control_high = [
            (start, end) for value, start, end in segments(rows, args.non_control_channel) if value == 1
        ]

    full_control_passes = [end - start for start, end in whole_passes if overlaps((start, end), control_high)]
    full_no_control_passes = [end - start for start, end in whole_passes if not overlaps((start, end), control_high)]
    control_body = [end - start for start, end in control_high]
    control_periods = [
        next_start - start for (start, _), (next_start, _) in zip(control_high, control_high[1:])
    ]

    summarize("full pass WITH control", full_control_passes, args.budget_us)
    summarize("full pass WITHOUT control", full_no_control_passes, args.budget_us)
    summarize("inner control body", control_body, args.budget_us)
    if non_control_high:
        non_control_work = [end - start for start, end in non_control_high]
        summarize("GP22 non-control-work segments", non_control_work, args.budget_us)
    summarize("control rising-edge period", control_periods, args.budget_us)


if __name__ == "__main__":
    main()
