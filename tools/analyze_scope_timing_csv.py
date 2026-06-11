#!/usr/bin/env python3
"""Analyze Saleae digital CSV exports from the Pico scope-timing build."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

DEFAULT_BUDGETS_US = [300.0, 312.5, 333.333]


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    k = (len(ordered) - 1) * q / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - k) + ordered[hi] * (k - lo)


def summarize(
    name: str, values: list[float], budgets_us: list[float], *, show_rate: bool = False
) -> None:
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
    if show_rate:
        print(f"  rate_from_mean={1.0 / statistics.fmean(values):.3f}Hz")


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


def overlap_count(
    intervals: list[tuple[float, float]], candidates: list[tuple[float, float]]
) -> int:
    count = 0
    candidate_index = 0
    for start, end in intervals:
        while candidate_index < len(candidates) and candidates[candidate_index][1] <= start:
            candidate_index += 1
        if candidate_index < len(candidates) and candidates[candidate_index][0] < end:
            count += 1
    return count


def resolve_channel(spec: str | None, channel_names: list[str], label: str) -> int | None:
    if spec is None or spec.strip().lower() in ("none", "off", "-"):
        return None
    normalized = spec.strip()
    if normalized in channel_names:
        return channel_names.index(normalized)
    channel_name = f"Channel {normalized}"
    if channel_name in channel_names:
        return channel_names.index(channel_name)
    try:
        column_index = int(normalized)
    except ValueError as exc:
        raise SystemExit(f"{label} channel {spec!r} is not in CSV header {channel_names}") from exc
    if 0 <= column_index < len(channel_names):
        return column_index
    raise SystemExit(f"{label} channel {spec!r} is outside CSV header {channel_names}")


def high_intervals(rows: list[tuple[float, list[int]]], channel_index: int) -> list[tuple[float, float]]:
    return [(start, end) for value, start, end in segments(rows, channel_index) if value == 1]


def rising_edges(rows: list[tuple[float, list[int]]], channel_index: int) -> list[float]:
    return [start for value, start, _ in segments(rows, channel_index) if value == 1]


def clustered_edges(edges: list[float], quiet_us: float) -> list[float]:
    quiet_s = quiet_us * 1e-6
    clustered: list[float] = []
    last: float | None = None
    for edge in edges:
        if last is None or edge - last > quiet_s:
            clustered.append(edge)
        last = edge
    return clustered


def periods(edges: list[float]) -> list[float]:
    return [next_edge - edge for edge, next_edge in zip(edges, edges[1:])]


def drdy_to_control_latency(
    drdy_edges: list[float], control_edges: list[float], max_latency_us: float
) -> tuple[list[float], int]:
    import bisect

    max_latency_s = max_latency_us * 1e-6
    latencies: list[float] = []
    missed = 0
    for edge in drdy_edges:
        index = bisect.bisect_left(control_edges, edge)
        if index < len(control_edges) and control_edges[index] - edge <= max_latency_s:
            latencies.append(control_edges[index] - edge)
        else:
            missed += 1
    return latencies, missed


def drdy_to_control_completion_latency(
    drdy_edges: list[float], control_intervals: list[tuple[float, float]], max_latency_us: float
) -> tuple[list[float], int]:
    import bisect

    starts = [start for start, _ in control_intervals]
    max_latency_s = max_latency_us * 1e-6
    latencies: list[float] = []
    missed = 0
    for edge in drdy_edges:
        index = bisect.bisect_left(starts, edge)
        if index < len(control_intervals):
            _, end = control_intervals[index]
            if end - edge <= max_latency_s:
                latencies.append(end - edge)
                continue
        missed += 1
    return latencies, missed


def edges_inside_intervals(edges: list[float], intervals: list[tuple[float, float]]) -> int:
    import bisect

    count = 0
    for start, end in intervals:
        count += max(0, bisect.bisect_left(edges, end) - bisect.bisect_right(edges, start + 1e-6))
    return count


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
    parser.add_argument(
        "--gp14-channel",
        default="1",
        help="Saleae channel label/column for GP14 raw IMU DRDY, default: Channel 1; use 'none' if absent",
    )
    parser.add_argument(
        "--gp19-channel",
        default="0",
        help="Saleae channel label/column for GP19 control closure, default: Channel 0",
    )
    parser.add_argument(
        "--gp22-channel",
        default="3",
        help="Saleae channel label/column for GP22 selected diagnostic output, default: Channel 3; use 'none' if absent",
    )
    parser.add_argument(
        "--aux-channel",
        default=None,
        help="Optional extra channel label/column to summarize as an auxiliary digital signal",
    )
    parser.add_argument(
        "--aux-label",
        default="auxiliary signal",
        help="Name to use for --aux-channel summaries",
    )
    parser.add_argument(
        "--drdy-cluster-us",
        type=float,
        default=20.0,
        help="Quiet time used to deglitch/cluster GP14 DRDY edges.",
    )
    parser.add_argument(
        "--latency-window-us",
        type=float,
        default=1000.0,
        help="Maximum GP14-to-GP19 latency counted as matched.",
    )
    parser.add_argument(
        "--legacy-whole-channel",
        default=None,
        help="Optional old GP18 whole-pass channel label/column for legacy captures.",
    )
    parser.add_argument(
        "--whole-mode",
        choices=("toggle", "pulse"),
        default="toggle",
        help="GP18 mode: pass-boundary toggle for new captures, high pulse for older captures",
    )
    parser.add_argument(
        "--budget-us",
        type=float,
        action="append",
        default=None,
        help="Loop budget in microseconds; repeat for multiple budgets",
    )
    args = parser.parse_args()
    budgets_us = args.budget_us or DEFAULT_BUDGETS_US

    channel_names, rows = load_rows(args.csv)
    print(f"file={args.csv} rows={len(rows)} duration={rows[-1][0] - rows[0][0]:.6f}s")
    print(f"channels={', '.join(channel_names)}")

    gp14_index = resolve_channel(args.gp14_channel, channel_names, "GP14")
    gp19_index = resolve_channel(args.gp19_channel, channel_names, "GP19")
    gp22_index = resolve_channel(args.gp22_channel, channel_names, "GP22")
    aux_index = resolve_channel(args.aux_channel, channel_names, "aux")
    print(
        "pin mapping: "
        f"GP14={channel_names[gp14_index] if gp14_index is not None else 'absent'} "
        f"GP19={channel_names[gp19_index]} "
        f"GP22={channel_names[gp22_index] if gp22_index is not None else 'absent'}"
        f"{' ' + args.aux_label + '=' + channel_names[aux_index] if aux_index is not None else ''}"
    )

    gp19_high = high_intervals(rows, gp19_index)
    gp19_edges = [start for start, _ in gp19_high]

    if gp14_index is not None:
        gp14_edges = clustered_edges(rising_edges(rows, gp14_index), args.drdy_cluster_us)
        summarize("GP14 raw DRDY rising-edge period", periods(gp14_edges), budgets_us, show_rate=True)
    else:
        gp14_edges = []
    summarize("GP19 control closure pulse width", [end - start for start, end in gp19_high], budgets_us)
    summarize(
        "GP19 control closure rising-edge period",
        periods(gp19_edges),
        budgets_us,
        show_rate=True,
    )
    if gp22_index is not None:
        gp22_high = high_intervals(rows, gp22_index)
        summarize("GP22 selected diagnostic pulse width", [end - start for start, end in gp22_high], budgets_us)
    else:
        gp22_high = []

    if aux_index is not None:
        aux_high = high_intervals(rows, aux_index)
        aux_edges = rising_edges(rows, aux_index)
        summarize(f"{args.aux_label} high pulse width", [end - start for start, end in aux_high], budgets_us)
        summarize(f"{args.aux_label} rising-edge period", periods(aux_edges), budgets_us, show_rate=True)

    if gp14_index is not None:
        latencies, missed = drdy_to_control_latency(gp14_edges, gp19_edges, args.latency_window_us)
        summarize("GP14 DRDY to next GP19 control-start latency", latencies, budgets_us)
        completion_latencies, completion_missed = drdy_to_control_completion_latency(
            gp14_edges, gp19_high, args.latency_window_us
        )
        summarize(
            "GP14 DRDY to GP19 control-complete latency",
            completion_latencies,
            budgets_us,
        )
        print(
            "correlation: "
            f"gp14_events={len(gp14_edges)} gp19_pulses={len(gp19_high)} "
            f"delta={len(gp14_edges) - len(gp19_high)} "
            f"unmatched_gp14_within_{args.latency_window_us:.0f}us={missed} "
            f"unmatched_gp14_to_complete_within_{args.latency_window_us:.0f}us={completion_missed} "
            f"extra_gp14_while_gp19_high={edges_inside_intervals(gp14_edges, gp19_high)} "
            f"gp19_overlapping_gp22={overlap_count(gp19_high, gp22_high)}/{len(gp19_high)}"
        )

    whole_index = resolve_channel(args.legacy_whole_channel, channel_names, "legacy whole")
    if whole_index is None:
        return

    whole_segments = segments(rows, whole_index)
    if args.whole_mode == "toggle":
        whole_passes = [(start, end) for _, start, end in whole_segments if end > start]
    else:
        whole_passes = [(start, end) for value, start, end in whole_segments if value == 1]
    full_control_passes = [
        end - start for start, end in whole_passes if overlaps((start, end), gp19_high)
    ]
    full_no_control_passes = [
        end - start for start, end in whole_passes if not overlaps((start, end), gp19_high)
    ]

    summarize("legacy full pass WITH control", full_control_passes, budgets_us)
    summarize("legacy full pass WITHOUT control", full_no_control_passes, budgets_us)


if __name__ == "__main__":
    main()
