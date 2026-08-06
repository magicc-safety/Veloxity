# Pixracer Pro runtime sensor diagnostics

The `runtime-diagnostics` feature instruments the live sensor path without changing
the normal release firmware. It counts sensor publication, driver errors, replacement
inside last-value signals, flight-loop consumption and age, processor input/output,
replacement before telemetry, per-stream telemetry sends, scheduler timing, and USB
enqueue/dequeue errors. RC input uses a bounded eight-frame FIFO; diagnostics also
measure its maximum occupancy and any time the producer waits for room.

## Build and flash the diagnostic firmware

Connect the ST-Link probe and Pixracer Pro, then run from the Veloxity repository:

```bash
cargo xtask flash-board pixracerpro --vcp --runtime-diagnostics
```

The flash command downloads, verifies, resets, and detaches. Connect `rosflight_io`
over USB before taking a transport measurement. The existing ROSflight workspace must
already be sourced by the caller; Veloxity does not source or modify it.

This is the one-command opt-in step: `--runtime-diagnostics` builds an optimized release
image containing all runtime counters and flashes it. It includes IMU, magnetometer,
barometer, pitot, range, GNSS, battery, RC, scheduler, telemetry, and transport
instrumentation together; there is no separate magnetometer diagnostic feature to
enable. Capturing and analyzing are intentionally separate commands so the operator can
establish the desired ROS connection and physical test conditions after flashing.

## Capture a run

Leave the ST-Link connected. The capture tool reads counters without reflashing or
resetting the board and saves both absolute snapshots and their wrap-safe deltas:

```bash
python3 tools/capture_runtime_diagnostics.py --duration 60
```

Records are written under `runtime_diagnostics_runs/`, which is intentionally ignored
by Git. Use `--output path.json` to select another location. A useful test should have
the same physical connections and telemetry parameters at both ends of a comparison.

The JSON is the durable raw record. Capture and analysis are separate, so the same data
can be reused without touching the board:

```bash
# Reprint one run
python3 tools/analyze_runtime_diagnostics.py summary runtime_diagnostics_runs/run.json

# Mean, sample standard deviation, minimum, and maximum across repeated runs
python3 tools/analyze_runtime_diagnostics.py stats runtime_diagnostics_runs/*.json

# Field-by-field absolute and percentage changes
python3 tools/analyze_runtime_diagnostics.py compare before.json after.json

# One tidy row per run and sensor for pandas, R, MATLAB, or a spreadsheet
python3 tools/analyze_runtime_diagnostics.py export-csv \
  runtime_diagnostics_runs/*.json --output sensor_runs.csv
```

The records also retain every named counter in `before`, `after`, and `deltas`, so you
can calculate metrics not anticipated by the bundled analyzer. For example:

```bash
jq '.observed.VELOXITY_DIAG_IMU_TICK_MAX_US' run.json
```

Accumulating counters are reported as wrap-safe deltas. Maximum/minimum counters are
boot-wide gauges and are stored in `observed` using their ending absolute value.

Attaching through SWD briefly pauses the MCU. The baseline read can therefore create a
small, recognizable discontinuity immediately at the beginning of the measured window
(usually visible in BMI sensor-gap and max-age counters). It cannot create a sustained
rate mismatch, and longer runs make this fixed boundary effect negligible. Do not treat
one boundary gap as ordinary flight behavior; compare ongoing overwrite rates and repeat
the run when investigating a rare event.

Interpret each sensor row from left to right:

- `published` is a completed result offered by the driver task.
- `signal_overwrites` means a newer result replaced one not yet consumed.
- `queue_depth_max` is the greatest observed occupancy of a FIFO-backed sensor handoff.
- `queue_full_waits` means the FIFO filled and its producer yielded until the consumer
  made room. `queue_wait_avg_us` and `queue_wait_max_us` quantify that backpressure.
- `consumed` is the result removed by the flight loop.
- `processed_out` is a usable calibrated/passthrough packet produced by core.
- `unsent_overwrites` means a newer processed packet replaced one before its telemetry stream sent it.
- `telemetry_sent` means core selected and packed the sensor stream. Global VCP rejection,
  partial-frame, and USB error counters determine whether transport accepted those frames.

An error publication can legitimately make `processed_out` smaller than `consumed`.
Configured telemetry rate limiting can legitimately produce unsent replacement when the
sensor rate is higher than its `TEL_*_HZ` rate; with `TEL_*_HZ=0`, every new sample is
eligible and replacement indicates scheduling or transport pressure.

RC differs from the last-value sensor signals: its FIFO preserves an ordered burst of
up to eight decoded frames. If it ever fills, the producer waits rather than replacing
an older frame. Therefore `published == consumed` (apart from a one-frame live-snapshot
boundary) demonstrates lossless RC handoff; nonzero queue waits indicate that capacity
or consumer cadence deserves review but do not themselves mean a frame was lost.

The IST8308 magnetometer additionally reports `conversion_commands`, `drdy_ready`,
`drdy_misses`, and `i2c_errors`. Its Pixracer Pro connection has no routed DRDY GPIO,
so firmware matches ROSflight C by commanding one conversion per 10 ms period and
reading its status at a fixed phase. In a healthy run, commands, ready results,
publications, processing, and telemetry should match apart from a one-sample live
snapshot boundary; a DRDY miss directly explains a missing publication.

### IST8308 single-conversion timing

The Pixracer Pro IST8308 connection provides I2C but does not route the sensor's DRDY
output to an MCU GPIO. ROSflight C therefore does not use a magnetometer interrupt. Its
driver advances a state machine from a 10 kHz polling timer during every 10 ms sample
period:

1. At 0.0 ms, write `CNTL2=0x01` to command one conversion.
2. At 8.9 ms, select the `STAT1` register and record the sample timestamp.
3. At 9.2 ms, read status plus the six magnetic-field bytes.
4. Publish only when status is exactly the DRDY value (`0x01`).

Veloxity uses the same phases asynchronously. The previous implementation instead put
the IST8308 in continuous 100 Hz mode and polled it every 10 ms. The MCU timer and the
sensor's internal oscillator were independent, so some polls landed just before a new
conversion completed. Those polls correctly rejected stale data but reduced publication
to approximately 96--97 Hz.

Interpret the new counters as follows:

- `conversion_commands` counts successfully issued single-conversion commands.
- `drdy_ready` counts reads that returned a complete new sample.
- `drdy_misses` counts reads where conversion was not ready; no stale sample is published.
- `i2c_errors` counts command, register-selection, or data-read failures in acquisition.

For a normal interval, expect `conversion_commands == drdy_ready == published ==
consumed == processed_out == telemetry_sent`, allowing for one item crossing the
non-atomic live snapshot boundary. A nonzero `drdy_misses` value explains the same number
of absent publications; it is not a telemetry or service-loop loss.

The capture command prints an `IST8308 acquisition` summary automatically and retains
the counters in JSON for `summary`, `stats`, `compare`, and `export-csv` analysis. For
example:

```bash
cargo xtask flash-board pixracerpro --vcp --runtime-diagnostics
# Start exactly one rosflight_io instance after the board reconnects.
python3 tools/capture_runtime_diagnostics.py --duration 60 \
  --output runtime_diagnostics_runs/mag_single_conversion.json
python3 tools/analyze_runtime_diagnostics.py summary \
  runtime_diagnostics_runs/mag_single_conversion.json
```

SWD counter reads briefly halt the MCU. If a rare single DRDY miss appears at a capture
boundary, do not immediately classify it as normal sensor behavior. Check the undisturbed
host stream with the probe idle:

```bash
ros2 topic hz /magnetometer
```

For stronger evidence, power-cycle or reset the diagnostic firmware, leave the probe
idle for several minutes, and then take one cumulative debugger snapshot. If misses grow
during undisturbed operation, investigate delayed I2C command completion or increase the
wait relative to the actual completed command. If they only appear when SWD attaches,
they are measurement artifacts.

The report also prints every nonzero counter containing `OVERWRITE`, `REJECTED`,
`ERROR`, `MISSING`, `GAP`, or `PARTIAL`. Examine the saved JSON for timing totals and
maxima such as IMU tick, service phase, telemetry drain, acquisition age, BMI088 sample
time, battery ADC wall time, and VCP byte/packet throughput.

## Flash normal release firmware

Do not pass `--runtime-diagnostics`:

```bash
cargo xtask flash-board pixracerpro --vcp
```

All diagnostic atomics and recording branches are guarded with Cargo features and are
absent from this build. Confirm with:

```bash
arm-none-eabi-nm target/thumbv7em-none-eabihf/release/veloxity | rg VELOXITY_DIAG
```

No output is expected for a normal release build.
