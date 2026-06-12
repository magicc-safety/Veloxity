#!/usr/bin/env python3
"""Plot Saleae timing distributions for Voloxide scope-timing captures."""

from __future__ import annotations

import argparse
import bisect
import csv
import statistics
from pathlib import Path

import matplotlib.pyplot as plt


def load_rows(path: Path) -> tuple[list[str], list[tuple[float, list[int]]]]:
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        rows = [(float(row[0]), [int(value) for value in row[1:]]) for row in reader]
    if not rows:
        raise SystemExit(f"{path} has no samples")
    return header[1:], rows


def resolve_channel(spec: str, channel_names: list[str]) -> int:
    if spec in channel_names:
        return channel_names.index(spec)
    channel_name = f"Channel {spec}"
    if channel_name in channel_names:
        return channel_names.index(channel_name)
    index = int(spec)
    if 0 <= index < len(channel_names):
        return index
    raise SystemExit(f"channel {spec!r} is outside CSV header {channel_names}")


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


def high_intervals(rows: list[tuple[float, list[int]]], channel_index: int) -> list[tuple[float, float]]:
    return [(start, end) for value, start, end in segments(rows, channel_index) if value == 1]


def rising_edges(rows: list[tuple[float, list[int]]], channel_index: int) -> list[float]:
    return [start for value, start, _ in segments(rows, channel_index) if value == 1]


def clustered_edges(edges: list[float], quiet_us: float) -> list[float]:
    quiet_s = quiet_us * 1e-6
    out: list[float] = []
    last: float | None = None
    for edge in edges:
        if last is None or edge - last > quiet_s:
            out.append(edge)
        last = edge
    return out


def periods(edges: list[float]) -> list[float]:
    return [next_edge - edge for edge, next_edge in zip(edges, edges[1:])]


def matched_deadline_latencies(
    deadline_edges: list[float], control_intervals: list[tuple[float, float]], max_latency_us: float
) -> tuple[list[float], list[float]]:
    control_starts = [start for start, _ in control_intervals]
    max_latency_s = max_latency_us * 1e-6
    start_latencies: list[float] = []
    completion_latencies: list[float] = []
    for deadline in deadline_edges:
        index = bisect.bisect_left(control_starts, deadline)
        if index >= len(control_intervals):
            continue
        start, end = control_intervals[index]
        if start - deadline <= max_latency_s:
            start_latencies.append(start - deadline)
            completion_latencies.append(end - deadline)
    return start_latencies, completion_latencies


def describe_us(values_s: list[float]) -> str:
    if not values_s:
        return "n=0"
    values_us = sorted(value * 1e6 for value in values_s)

    def pct(q: float) -> float:
        k = (len(values_us) - 1) * q / 100.0
        lo = int(k)
        hi = min(lo + 1, len(values_us) - 1)
        return values_us[lo] * (hi - k) + values_us[hi] * (k - lo)

    return (
        f"n={len(values_us)} mean={statistics.fmean(values_us):.2f}us "
        f"p50={pct(50):.2f}us p95={pct(95):.2f}us p99={pct(99):.2f}us "
        f"max={values_us[-1]:.2f}us"
    )


def plot_hist(ax, values_s: list[float], title: str, xlabel: str, bins: int = 120) -> None:
    values_us = [value * 1e6 for value in values_s]
    ax.hist(values_us, bins=bins, color="#336699", alpha=0.85)
    ax.set_title(f"{title}\n{describe_us(values_s)}")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.25)


def plot_timeseries(ax, edges: list[float], values_s: list[float], title: str, ylabel: str) -> None:
    if not values_s:
        ax.set_title(title)
        return
    start = edges[0] if edges else 0.0
    x = [(edge - start) for edge in edges[: len(values_s)]]
    y = [value * 1e6 for value in values_s]
    ax.plot(x, y, linewidth=0.8, color="#444444")
    ax.set_title(title)
    ax.set_xlabel("capture time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)


def save_distribution_grid(out_dir: Path, metrics: list[tuple[str, str, list[float]]]) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(15, 13))
    for ax, (title, xlabel, values) in zip(axes.flat, metrics):
        plot_hist(ax, values, title, xlabel)
    for ax in axes.flat[len(metrics) :]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(out_dir / "scope_timing_distributions.png", dpi=160)
    plt.close(fig)


def save_timeseries_grid(out_dir: Path, series: list[tuple[str, str, list[float], list[float]]]) -> None:
    fig, axes = plt.subplots(len(series), 1, figsize=(15, max(8, 2.7 * len(series))), sharex=False)
    if len(series) == 1:
        axes = [axes]
    for ax, (title, ylabel, edges, values) in zip(axes, series):
        plot_timeseries(ax, edges, values, title, ylabel)
    fig.tight_layout()
    fig.savefig(out_dir / "scope_timing_timeseries.png", dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("scope_plots"))
    parser.add_argument("--gp19-channel", default="0")
    parser.add_argument("--gp14-channel", default="1")
    parser.add_argument("--gp18-channel", default="2")
    parser.add_argument("--gp22-channel", default="3")
    parser.add_argument("--drdy-cluster-us", type=float, default=20.0)
    args = parser.parse_args()

    channel_names, rows = load_rows(args.csv)
    gp19 = resolve_channel(args.gp19_channel, channel_names)
    gp14 = resolve_channel(args.gp14_channel, channel_names)
    gp18 = resolve_channel(args.gp18_channel, channel_names)
    gp22 = resolve_channel(args.gp22_channel, channel_names)

    gp14_edges = clustered_edges(rising_edges(rows, gp14), args.drdy_cluster_us)
    gp18_edges = rising_edges(rows, gp18)
    gp19_high = high_intervals(rows, gp19)
    gp19_edges = [start for start, _ in gp19_high]
    gp22_high = high_intervals(rows, gp22)
    gp22_edges = [start for start, _ in gp22_high]

    gp14_periods = periods(gp14_edges)
    gp18_periods = periods(gp18_edges)
    gp19_periods = periods(gp19_edges)
    gp19_widths = [end - start for start, end in gp19_high]
    gp22_widths = [end - start for start, end in gp22_high]
    deadline_to_start, deadline_to_complete = matched_deadline_latencies(gp18_edges, gp19_high, 1500.0)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    metrics = [
        ("IMU DRDY period", "period (us)", gp14_periods),
        ("Control deadline period", "period (us)", gp18_periods),
        ("Control GP19 period", "period (us)", gp19_periods),
        ("Control body width", "width (us)", gp19_widths),
        ("Deadline to control start", "latency (us)", deadline_to_start),
        ("Deadline to control complete", "latency (us)", deadline_to_complete),
    ]
    save_distribution_grid(args.out_dir, metrics)

    series = [
        ("IMU DRDY period over time", "period (us)", gp14_edges, gp14_periods),
        ("Control deadline period over time", "period (us)", gp18_edges, gp18_periods),
        ("Control GP19 period over time", "period (us)", gp19_edges, gp19_periods),
        ("Control body width over time", "width (us)", gp19_edges, gp19_widths),
        ("Deadline to control complete over time", "latency (us)", gp18_edges, deadline_to_complete),
        ("Service GP22 width over time", "width (us)", gp22_edges, gp22_widths),
    ]
    save_timeseries_grid(args.out_dir, series)

    summary_path = args.out_dir / "scope_timing_summary.txt"
    with summary_path.open("w") as handle:
        handle.write(f"source={args.csv}\n")
        handle.write(f"duration_s={rows[-1][0] - rows[0][0]:.6f}\n")
        for title, _, values in metrics:
            handle.write(f"{title}: {describe_us(values)}\n")
        handle.write(f"Service GP22 width: {describe_us(gp22_widths)}\n")

    print(f"wrote {args.out_dir / 'scope_timing_distributions.png'}")
    print(f"wrote {args.out_dir / 'scope_timing_timeseries.png'}")
    print(f"wrote {summary_path}")


if __name__ == "__main__":
    main()
