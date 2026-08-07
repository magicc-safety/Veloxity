#!/usr/bin/env python3
"""Capture and report feature-gated Veloxity counters from a running board."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import re
import subprocess
import time

PREFIX = "VELOXITY_DIAG_"


def command(*args: str) -> str:
    return subprocess.run(args, check=True, text=True, capture_output=True).stdout


def symbols(elf: pathlib.Path, nm: str) -> dict[str, int]:
    found: dict[str, int] = {}
    for line in command(nm, "--numeric-sort", str(elf)).splitlines():
        fields = line.split()
        if len(fields) >= 3 and fields[-1].startswith(PREFIX):
            found[fields[-1]] = int(fields[0], 16)
    if not found:
        raise SystemExit(
            f"no {PREFIX} symbols in {elf}; build/flash with --runtime-diagnostics"
        )
    return found


def groups(addresses: list[int], maximum_gap: int = 256) -> list[tuple[int, int]]:
    result: list[tuple[int, int]] = []
    start = previous = addresses[0]
    for address in addresses[1:]:
        if address - previous > maximum_gap:
            result.append((start, previous))
            start = address
        previous = address
    result.append((start, previous))
    return result


def snapshot(symbol_table: dict[str, int], probe: str, chip: str) -> dict[str, int]:
    memory: dict[int, int] = {}
    for start, end in groups(sorted(symbol_table.values())):
        words = (end - start) // 4 + 1
        output = command(
            probe, "read", "--chip", chip, "b32", hex(start), str(words)
        )
        values = [
            int(value, 16)
            for line in output.splitlines()
            for value in re.findall(
                r"\b[0-9a-fA-F]{8}\b", line.split(":", 1)[-1]
            )
        ]
        if len(values) != words:
            raise SystemExit(
                f"expected {words} words at {start:#x}, received {len(values)}"
            )
        memory.update((start + 4 * index, value) for index, value in enumerate(values))
    return {name: memory[address] for name, address in symbol_table.items()}


def delta(before: dict[str, int], after: dict[str, int]) -> dict[str, int]:
    return {name: (after[name] - value) & 0xFFFF_FFFF for name, value in before.items()}


def observed(delta_values: dict[str, int], after: dict[str, int]) -> dict[str, int]:
    """Use deltas for accumulators and absolute values for boot-wide gauges."""
    values = dict(delta_values)
    for name, value in after.items():
        if (
            "_MAX_" in name
            or name.endswith("_MAX")
            or "_MIN_" in name
            or name.endswith("_MIN")
        ):
            values[name] = value
    return values


def sensor_rows(values: dict[str, int], seconds: float) -> list[dict[str, object]]:
    rows = []
    for sensor in ("BMI", "MAG", "BARO", "PITOT", "RANGE", "GNSS", "BATTERY", "RC", "PPS"):
        key = lambda suffix: values.get(f"{PREFIX}{sensor}_{suffix}")  # noqa: E731
        publish = key("SIGNAL_PUBLISH") if sensor == "BMI" else key("PUBLISH")
        if publish is None:
            continue
        row: dict[str, object] = {
            "sensor": "IMU" if sensor == "BMI" else sensor,
            "published": publish,
            "publish_hz": publish / seconds,
            "errors": key("ERROR_PUBLISH") or 0,
            "signal_overwrites": key("SIGNAL_OVERWRITE") or 0,
        }
        queue_full_waits = key("QUEUE_FULL_WAITS")
        if queue_full_waits is not None:
            queue_wait_sum_us = key("QUEUE_WAIT_SUM_US") or 0
            row["queue_full_waits"] = queue_full_waits
            row["queue_wait_avg_us"] = (
                queue_wait_sum_us / queue_full_waits if queue_full_waits else 0.0
            )
            row["queue_wait_max_us"] = key("QUEUE_WAIT_MAX_US") or 0
            row["queue_depth_max"] = key("QUEUE_DEPTH_MAX") or 0
        if sensor == "MAG" and key("CONVERSION_COMMAND") is not None:
            row["conversion_commands"] = key("CONVERSION_COMMAND") or 0
            row["drdy_ready"] = key("DRDY_READY") or 0
            row["drdy_misses"] = key("DRDY_MISS") or 0
            row["i2c_errors"] = key("I2C_ERROR") or 0
        consume_sensor = "IMU" if sensor == "BMI" else sensor
        for label, suffix in (
            ("consumed", "CONSUME"),
            ("consume_errors", "CONSUME_ERROR"),
            ("processed_in", "PROCESS_INPUT"),
            ("processed_out", "PROCESS_OUTPUT"),
            ("unsent_overwrites", "UNSENT_OVERWRITE"),
        ):
            value = values.get(f"{PREFIX}{consume_sensor}_{suffix}")
            if value is not None:
                row[label] = value
                if label in ("consumed", "processed_in", "processed_out", "telemetry_sent"):
                    row[f"{label}_hz"] = value / seconds
        sent = values.get(f"{PREFIX}TELEM_{consume_sensor}_SENT")
        if sent is not None:
            row["telemetry_sent"] = sent
        age_sum = values.get(f"{PREFIX}{consume_sensor}_CONSUME_AGE_SUM_US")
        age_max = values.get(f"{PREFIX}{consume_sensor}_CONSUME_AGE_MAX_US")
        consumed = row.get("consumed", 0)
        if age_sum is not None and isinstance(consumed, int) and consumed:
            row["consume_age_avg_us"] = age_sum / consumed
            row["consume_age_max_us"] = age_max or 0
        rows.append(row)
    return rows


def print_scheduler_transport(values: dict[str, int]) -> None:
    """Print optional counters added after the original sensor report format."""
    armed_imu_count = values.get(f"{PREFIX}ARMED_IMU_TICK_COUNT")
    armed_service_count = values.get(f"{PREFIX}ARMED_SERVICE_PHASE_COUNT")
    if armed_imu_count is not None or armed_service_count is not None:
        imu_sum = values.get(f"{PREFIX}ARMED_IMU_TICK_SUM_US", 0)
        service_sum = values.get(f"{PREFIX}ARMED_SERVICE_PHASE_SUM_US", 0)
        print("\nArmed scheduler headroom")
        print(
            f"  imu_ticks={armed_imu_count or 0} "
            f"avg_us={imu_sum / armed_imu_count if armed_imu_count else 0.0:.1f} "
            f"max_us={values.get(f'{PREFIX}ARMED_IMU_TICK_MAX_US', 0)}"
        )
        print(
            f"  service_phases={armed_service_count or 0} "
            f"avg_us={service_sum / armed_service_count if armed_service_count else 0.0:.1f} "
            f"max_us={values.get(f'{PREFIX}ARMED_SERVICE_PHASE_MAX_US', 0)}"
        )

    requests = values.get(f"{PREFIX}TIMESYNC_REQUEST_RECEIVED")
    if requests is not None:
        print("\nTIMESYNC handling")
        print(
            f"  requests={requests} "
            f"overwrites={values.get(f'{PREFIX}TIMESYNC_REQUEST_OVERWRITE', 0)} "
            f"responses={values.get(f'{PREFIX}TIMESYNC_RESPONSE_SENT', 0)}"
        )

    rx_packets = values.get(f"{PREFIX}VCP_RX_USB_PACKETS")
    if rx_packets is not None:
        waits = values.get(f"{PREFIX}VCP_RX_WAIT_COUNT", 0)
        wait_sum = values.get(f"{PREFIX}VCP_RX_WAIT_SUM_US", 0)
        print("\nVCP receive path")
        print(
            f"  usb_packets={rx_packets} "
            f"usb_bytes={values.get(f'{PREFIX}VCP_RX_USB_BYTES', 0)} "
            f"pipe_min_free={values.get(f'{PREFIX}VCP_RX_PIPE_MIN_FREE', 0)}"
        )
        print(
            f"  pipe_waits={waits} "
            f"wait_avg_us={wait_sum / waits if waits else 0.0:.1f} "
            f"wait_max_us={values.get(f'{PREFIX}VCP_RX_WAIT_MAX_US', 0)}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--elf", type=pathlib.Path, default=pathlib.Path("target/thumbv7em-none-eabihf/release/veloxity"))
    parser.add_argument("--chip", default="STM32H743IIKx")
    parser.add_argument("--probe-rs", default="probe-rs")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()

    table = symbols(args.elf, args.nm)
    print(
        f"Reading baseline for {len(table)} counters; measurement has not started...",
        flush=True,
    )
    before = snapshot(table, args.probe_rs, args.chip)
    print(
        f"Baseline complete; starting {args.duration:.1f}-second measurement now.",
        flush=True,
    )
    started = time.monotonic()
    time.sleep(args.duration)
    after = snapshot(table, args.probe_rs, args.chip)
    elapsed = time.monotonic() - started
    delta_values = delta(before, after)
    values = observed(delta_values, after)
    rows = sensor_rows(values, elapsed)

    print(f"\nSensor pipeline ({elapsed:.3f} s)")
    columns = ("sensor", "publish_hz", "published", "errors", "signal_overwrites", "queue_full_waits", "queue_depth_max", "consumed", "processed_out", "unsent_overwrites", "telemetry_sent")
    print(" ".join(f"{column:>18}" for column in columns))
    for row in rows:
        rendered = []
        for column in columns:
            value = row.get(column, "-")
            rendered.append(f"{value:18.3f}" if isinstance(value, float) else f"{str(value):>18}")
        print(" ".join(rendered))

    mag = next((row for row in rows if row["sensor"] == "MAG"), None)
    if mag is not None and "conversion_commands" in mag:
        print("\nIST8308 acquisition")
        print(
            f"  commands={mag['conversion_commands']} "
            f"drdy_ready={mag['drdy_ready']} "
            f"drdy_misses={mag['drdy_misses']} "
            f"i2c_errors={mag['i2c_errors']}"
        )

    print_scheduler_transport(values)

    print("\nNonzero loss/error counters")
    loss_words = ("OVERWRITE", "REJECTED", "ERROR", "MISSING", "GAP", "PARTIAL")
    losses = [(name, value) for name, value in sorted(delta_values.items()) if value and any(word in name for word in loss_words)]
    if losses:
        for name, value in losses:
            print(f"  {name}: {value}")
    else:
        print("  none")

    output = args.output
    if output is None:
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = pathlib.Path("runtime_diagnostics_runs") / f"pixracerpro_{stamp}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    record = {
        "captured_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "duration_seconds": elapsed,
        "elf": str(args.elf),
        "chip": args.chip,
        "sensors": rows,
        "deltas": dict(sorted(delta_values.items())),
        "observed": dict(sorted(values.items())),
        "before": dict(sorted(before.items())),
        "after": dict(sorted(after.items())),
    }
    output.write_text(json.dumps(record, indent=2) + "\n")
    print(f"\nSaved full counter record to {output}")


if __name__ == "__main__":
    main()
