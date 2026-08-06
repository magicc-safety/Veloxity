# Pixracer Pro C-parity and real-time work, 2026-08-05 through 2026-08-06

This is an engineering record of the work performed across the complete branch chain
leading to `Derekbenj/telemetry-queue-architecture`. It is deliberately not part of the
Veloxity documentation website navigation. The purpose is to preserve the decisions,
measurements, experiments, and unfinished risks that would otherwise be difficult to
reconstruct from individual commits.

The work began on `Derekbenj/sdcard_read_write` and accumulated through several
branches made from the preceding branch:

```text
Derekbenj/sdcard_read_write
  -> Derekbenj/gnss-hmsl
    -> Derekbenj/imu-axis-negation
      -> Derekbenj/fresh-sensor-telemetry
        -> Derekbenj/battery_voltage_addition
          -> Derekbenj/telemetry-queue-architecture
```

At the time of writing, the last committed ancestor of the current branch is
`b82fd21`. The scheduler, transport diagnostics, RC FIFO, async battery ADC,
single-conversion magnetometer, diagnostic tools, and their tests are still working-tree
changes. Preserve and commit those changes before treating this note as a description of
a remote branch.

## Branch and commit map

| Branch | Tip during this work | Principal change |
| --- | --- | --- |
| `Derekbenj/sdcard_read_write` | `0ad4392` | C-compatible SD parameter image and version identity |
| `Derekbenj/gnss-hmsl` | `3bf1bff` | GNSS MSL altitude and initial sensor parity work |
| `Derekbenj/imu-axis-negation` | `e98f8fe` | IMU orientation experiment and IMU temperature units |
| `Derekbenj/fresh-sensor-telemetry` | `b7aa4dd` | Fresh-only sensor telemetry and calibration parity |
| `Derekbenj/battery_voltage_addition` | `b82fd21` | Battery ADC, storage compatibility, final IMU mapping, and 100 Hz magnetometer default |
| `Derekbenj/telemetry-queue-architecture` | working tree after `b82fd21` | Slack-driven service, loss diagnostics, RC FIFO, async ADC, and deterministic magnetometer acquisition |

The ancestry commits, in order, are:

```text
0bac189  Merge origin/main into Derekbenj/sdcard_read_write
422308a  Store parameters in ROSflight C format
0ad4392  Report ROSflight-compatible Veloxity version
1035732  Match ROSflight C GNSS MSL altitude
7784802  Match Pixracer barometer telemetry rate
8cd7445  Match Pixracer sensor conventions
3bf1bff  Match ROSflight C magnetometer scaling
11bcffb  Match Pixracer C IMU axis orientation (later superseded)
e98f8fe  Report BMI088 temperature in kelvin
9e9db2d  Send telemetry only for fresh sensor samples
c6539c7  Match C barometer calibration status
81fddd9  Enforce IMU calibration interlock at startup
0d62107  Use board clock for barometer calibration
b7aa4dd  Use board clock for pitot calibration
f2e6448  Add Pixracer battery monitoring and storage compatibility
b82fd21  Match ROSflight C magnetometer telemetry rate
```

The headings below describe the final intended behavior, and explicitly identify an
experiment when a later decision superseded it.

## ROSflight C reference firmware and mixer incident

The C reference used by the end of this work is upstream `rosflight_firmware` `main` at
`a46527bd8e49d00a072c7efd7af9dd543910d831`. Its short hash is also the current SD
schema hash described below.

During the first battery comparisons, an older C image produced an invalid mixer or
enormous mixer coefficients. A temporary local patch changed pseudoinverse handling to
ignore effectively zero singular values. Upstream commit `a46527b`, "fix: remove exact
equality comparison in favor of svd.solve (#492)," now contains the permanent version:
it gives Eigen's SVD a floating-point threshold, solves the pseudoinverse through SVD,
and zeros inversion noise below `1e-6`. The final C reference was built from that clean
upstream commit with no local mixer patch. The timing-harness firmware and its scripts
were deliberately not used for this reference image.

`USE_MOTOR_PARAM` is not a Veloxity-only addition. It exists in both current ROSflight C
and Veloxity, with integer default zero, and is part of the compatible persisted schema.
It chooses the motor-parameter mixer path; it is separate from the pseudoinverse bug.

The C firmware repository used locally was:

```text
../rosflight/rosflight/workspace/src/rosflight_ros_pkgs/rosflight_firmware
```

For a reproducible C comparison, confirm that repository is clean except for its build
directory, confirm `git rev-parse HEAD` is the hash above, build the Pixracer Pro target,
and transfer/flash the resulting ELF. Do not apply the obsolete local mixer patch on top
of `a46527b`; that would make the reference differ from upstream main.

## ROSflight C-compatible SD parameter storage

Veloxity now encodes the parameter card image using the ROSflight 2.0 C firmware's ARM
`params_t` layout instead of a Rust-native structure. The persisted image is 7,004 bytes
and contains 333 C parameters, four-byte values, 16-byte names, one-byte type tags, the
schema hash, magic, and checksum in the same representation expected by C. This makes
the SD card directly readable after changing between C and Veloxity firmware.

The current C schema hash used by Veloxity is `0xA46527BD`. Veloxity also accepts the
previous compatible hash `0xC3A233B8` because the relevant layout did not change; the
next save rewrites the current hash. Decoding validates the complete expected sequence
of parameter names and types. It does not try to recognize one special incorrect order
from an old Rust implementation. A malformed or differently ordered image is rejected
unambiguously instead of being silently associated with the wrong parameter names.

On an invalid read, the current behavior is to report the storage failure, load defaults,
and write a new default image. That is an important operational detail: failure does not
leave the card untouched for a later manual wipe. The status message must remain visible
because overwriting a bad image can otherwise look like unexplained parameter loss.

### Parameters that are intentionally not on the C image

Veloxity has parameters that ROSflight C does not have. They remain usable at runtime
but are excluded from the C-compatible SD payload by the predicate in
`crates/veloxity_core/src/params/storage.rs`. The exclusions are:

- `CHN_OUTPUT_MASK`, whose Veloxity default is `-1`.
- `BYPASS_UNH_EST`.
- `EST_ANG_LOCKOUT`.
- `RC_KILL_CHN`, the output-kill RC channel used by Veloxity's RC switch handling.
- Every Veloxity telemetry-rate parameter: `TEL_HB_HZ`, `TEL_STATUS_HZ`,
  `TEL_IMU_HZ`, `TEL_ATT_HZ`, `TEL_OUT_HZ`, `TEL_DIFF_HZ`, `TEL_BARO_HZ`,
  `TEL_MAG_HZ`, `TEL_RANGE_HZ`, `TEL_BATT_HZ`, `TEL_GNSS_HZ`, and `TEL_RC_HZ`.

To keep another future Veloxity-only parameter off the SD card, add its `ParamId` to
that storage exclusion match and add a storage round-trip test. It is not enough to add
it to the parameter declaration alone.

These exclusions do not change hard-coded defaults. Startup creates the ordinary
Veloxity defaults, overlays the values decoded from the C image for C-compatible
parameters, and leaves the excluded Veloxity-only values at their local defaults. An
"overlay" here means calling the parameter API to replace the current in-memory value;
it does not rewrite the source-code default.

The version response was changed to `v2.0.0-veloxity-1.0`. This identifies the firmware
as Veloxity while retaining the ROSflight 2.0 compatibility prefix expected by the
unmodified `rosflight_io` node.

## GNSS altitude parity

The u-blox NAV-PVT packet provides both ellipsoidal height and height above mean sea
level. ROSflight C publishes the latter. Veloxity previously propagated ellipsoidal
height, creating a large, location-dependent vertical offset between otherwise matching
paths.

Veloxity now propagates NAV-PVT `hMSL` through the board packet, core GNSS state, and
MAVLink packing. The MAVLink field is still named `height`; its value now has the same
meaning and scale as the C firmware and therefore reaches ROS with C-compatible
semantics.

## Sensor units, axes, and calibration parity

### Barometer

The barometer driver returns Celsius, while the ROS/MAVLink-facing temperature field is
Kelvin. Rust had published approximately `41.42` into that Kelvin field while C reported
approximately `314.16`. Veloxity now adds `273.15` before telemetry packing.

The barometer's physical update rate is about 50 Hz. An early Veloxity configuration
attempted to publish it at 100 Hz, so approximately half of the ROS messages were exact
duplicates. Fresh-sample gating and the later sample-backed rate policy eliminate those
duplicates.

### Magnetometer

The IST8308 Z axis is inverted in the board-level conversion to match the ROSflight C
board convention. Its X scale is `1.515e-7 T/LSB`; Y and Z use `1.1515e-7 T/LSB`, which
matches the factors used by the C Pixracer driver rather than applying one nominal
datasheet factor to all three axes.

The first outside comparison found good settled calibration shapes in both firmwares:
C magnitude variation was about 0.505 percent and Veloxity about 0.451 percent. Before
the parity corrections, Veloxity reported roughly 47.1 microtesla versus C's 39.5
microtesla, with the difference largely explained by the driver scale factors, and its
Z sign was opposite C.

### BMI088 IMU

BMI088 temperature is now converted from Celsius to Kelvin before publication.

Commit `11bcffb` initially applied the C driver's `diag(-1, -1, +1)` mapping. Physical
testing showed that copying that transform obscured the distinction between the raw
sensor-to-board transform and the board-to-aircraft mounting transform. The final
production decision in the later battery/storage work supersedes it: the BMI088 board
mapping is identity, `[+1, +1, +1]`, because the raw axes already match the physical
Pixracer Pro board axes. Aircraft installation belongs in `IMU_ROLL`, `IMU_PITCH`, and
`IMU_YAW`.

For the tested installation—board upside down with its arrow forward—the coherent
Veloxity configuration is a 180-degree roll. A 180-degree pitch appeared necessary only
when the copied C X/Y negation was present. The transform is still right-handed: two
axis negations form a proper 180-degree rotation, but it was the wrong layer at which to
express the physical installation.

Attitude output can exist before a fresh calibration because an estimator can run on raw
IMU samples, but arming must not imply that an uncalibrated IMU is acceptable. Startup
now derives the uncalibrated-IMU error from the persisted calibration/bias parameters.
All-zero calibration values block arming; valid saved nonzero values are accepted. In
normal operation, reuse a known-good calibration only when the hardware, temperature,
and mounting remain trustworthy. Recalibrate after remounting, large temperature
changes, suspicious bias, or maintenance—not necessarily after every reboot.

## Fresh samples and telemetry rates

Sensor telemetry freshness is keyed by each packet's acquisition timestamp. Core tracks
the last timestamp sent for every sample-backed stream and sends a new packet once,
rather than repeatedly repacking the latest value just because telemetry service ran
again.

The current working tree sets sample-backed `TEL_*_HZ` defaults to zero. For those
streams, zero means "send every fresh sample," a positive value is an explicit maximum
rate, and `-1` disables the stream. Heartbeat, status, and raw-output telemetry retain
fixed defaults because they are not simple one-message-per-acquisition sensor streams.
All due streams are ordered by deadline; a numerical telemetry setting is no longer
used to manufacture duplicate sensor values.

The earlier `b82fd21` commit set the magnetometer default to 100 Hz. The current zero
default is its more general successor: a healthy 100 Hz magnetometer is still sent at
100 Hz, but the firmware follows actual completed conversions instead of an unrelated
periodic retransmission schedule.

## Calibration behavior and user messages

Barometer calibration now reports the same useful operator messages as C:

```text
Baro ground pressure cal successful!
Too much movement for barometer ground pressure cal
```

Successful calibration writes the pressure bias and `GROUND_LEVEL`. Higher estimator
states then use current pressure-derived height relative to that ground reference; they
are not merely relabeling absolute pressure as a relative measurement.

Barometer and differential-pressure/pitot calibration cadence now uses the board's
monotonic clock, matching the C firmware. This prevents a repeated, delayed, or
sensor-specific acquisition timestamp from changing the duration of the calibration
window. It does **not** make every barometer packet carry one fixed timestamp. Sensor
packets retain their acquisition times, and the shared ingestion context makes the board
clock available only to logic that needs elapsed wall time.

## Pixracer Pro battery monitoring

Pixracer Pro exposes battery voltage on ADC input PA2 and current on PA3. VREFINT is
sampled on ADC3. Veloxity now reads those channels and applies the ROSflight C-compatible
reference calibration plus `BATT_VOLT_MULT` and `BATT_CURR_MULT` parameter scaling.
Correct SD decoding was essential here: a malformed image or values assigned according
to the old Rust order could make the ADC look wrong even when the electrical input was
correct.

After correcting the power-port connection, SD representation, and scale values,
Veloxity measured about 23.7 V, agreeing with the external meter for the six-cell pack.
An observed current scale near `0.995` was plausible for that setup but is not a universal
flight calibration. Current should be calibrated against a trustworthy series current
measurement or known load over the expected operating range.

The current architecture replaces the blocking ADC/PAC transaction with Embassy async
DMA. ADC3/VREFINT uses DMA2 channel 1 and ADC1's PA2/PA3 sequence uses DMA2 channel 0,
with an 810.5-cycle sample time. A battery acquisition can span approximately 2.48 ms of
wall time, but the task yields while hardware and DMA are working instead of occupying
the CPU for that interval. Battery publication remains approximately 10 Hz.

## Why the service architecture was reworked

The former service phase admitted only two telemetry streams per call and used several
single latest-value handoffs. That design coupled throughput to a stream-count budget,
even when considerable CPU time remained before the next 400 Hz control deadline. A
new sample could replace an old one while service work was delayed, which initially
looked like a sensor-rate or interrupt problem.

The current scheduler uses a continuous, slack-driven service policy:

- Pending IMU/control work has priority.
- Service work checks a 200 microsecond guard before the next control deadline.
- Telemetry drain runs early in the service phase, before variable sensor, input, RC,
  log, and flush work.
- The guard is checked between streams and service stages instead of admitting an
  arbitrary number of messages.
- When no accumulated IMU sample can be processed, service can use the otherwise idle
  interval while still respecting the next deadline.
- Due telemetry is deadline ordered, so no one stream wins permanently by fixed list
  position.

A service phase around two milliseconds is not evidence of one two-millisecond blocking
call; it normally means the scheduler productively filled most of the available 2.5 ms
control period. The remaining rule is that each individual CPU-bound work unit must be
bounded or asynchronous, because a guard checked before a monolithic operation cannot
preempt that operation halfway through.

Embassy task priorities remain relevant. The IMU/control path is high priority; USB VCP
and the lower-rate sensor group have separate priorities. Several sensors share the
lower-rate priority, so a new always-ready, CPU-heavy task can still disturb its peers.
New sensors must be assessed for acquisition time, CPU time, bus contention, handoff
semantics, and telemetry/USB capacity rather than merely added to a service list.

One representative connected run measured the control loop near 399.08 Hz, with average
IMU/control work near 490 microseconds out of each 2,500-microsecond period and service
available after about 99.6 percent of ticks. A broader earlier run observed IMU 399.029
Hz, magnetometer 96.419 Hz, barometer 49.966 Hz, and GNSS/battery 10.006 Hz. That
magnetometer shortfall was later traced to acquisition phasing, not telemetry capacity.

## USB VCP transport

The upstream main ancestry already contains the independent concurrent VCP RX/TX fix
(`2352543`/`cf687d3`), and this work preserves it. `rosflight_io` remains unmodified.

Runtime instrumentation now counts frame attempts, enqueues, rejections, partial writes,
pipe dequeue, USB packets and bytes, USB errors, and minimum free capacity in the 2,048
byte transmit pipe. The minimum-free gauge reached zero at some point in boot-wide
history during testing, but the measured intervals had no rejected frames, partial
frames, or USB errors. A zero historical minimum is a warning to repeat the load test
when adding streams; by itself it does not prove a dropped message.

The rough transport cost measured for an ordinary telemetry message was about 47
microseconds of CPU service. A 100 Hz stream of similar size is therefore approximately
0.47 percent CPU, though packet size, USB scheduling, and simultaneous streams matter.

## RC burst preservation

RC originally used a one-slot latest-value `Signal`. In a 30.18-second diagnostic run,
6,696 frames were decoded at 221.863 Hz but only 5,962 were consumed, processed, and
sent; 734 were overwritten in the firmware handoff.

The handoff is now a bounded FIFO of eight decoded frames. With it, a comparable run
decoded and consumed all 6,696 frames at 221.828 Hz, with maximum occupancy 6/8 and no
full-queue waits. A second run with the transmitter/controller connected decoded and
consumed all 6,694 frames at 221.826 Hz, again without loss or full waits. Thus the
demonstrated loss was caused by Veloxity's one-slot handoff, not by whether the RC link
was connected.

The exact burst timing can still originate in SBUS framing, UART delivery, or task
phasing. The FIFO makes that normal burstiness safe and ordered. If it fills in a future
configuration, the producer waits instead of overwriting an older frame; sustained
waits would indicate upstream UART pressure and should be investigated using the queue
depth and wait-time counters.

Other sensor handoffs remain latest-value signals where that is intentional. If every
sample from another sensor becomes semantically important, it should receive a bounded
FIFO and the same production/consumption accounting rather than relying on latest-value
replacement.

## IST8308 acquisition parity

The Pixracer Pro routes IST8308 I2C but does not route its DRDY output to an MCU GPIO.
ROSflight C does not use a magnetometer interrupt on this board. Its polling state
machine commands a single conversion every 10 ms and samples it at fixed phases.

Veloxity previously used continuous 100 Hz mode and independently polled every 10 ms.
The MCU schedule and the sensor's internal oscillator drifted relative to each other, so
some polls occurred just before a new conversion and correctly rejected stale data. The
result was approximately 96–97 Hz even though telemetry had sufficient capacity.

Veloxity now mirrors the C single-conversion sequence asynchronously:

1. At 0.0 ms, write `CNTL2=0x01`.
2. At 8.9 ms, select `STAT1` and record the sample time.
3. At 9.2 ms, read status and the six measurement bytes.
4. Publish only when status is exactly `0x01`.

A clean 30.175-second diagnostic interval recorded 3,013 commands, 3,012 ready samples,
one not-ready result, zero I2C errors, and 3,012 publications/consumptions/telemetry
messages. The apparent 99.819 Hz includes debugger attachment and interval-boundary
effects. With diagnostics inactive and ROS receiving normally, `/magnetometer`
converged near 99.884 Hz, with nearly all intervals exactly 10 ms and one observed 20 ms
host-side gap.

One missed-ready count is not yet proven to be normal. It may come from the SWD capture
boundary, lower-priority task or I2C command phasing, or conversion-time variance. C
also checks DRDY and drops an incomplete conversion rather than publishing stale data.
A long reset/no-probe run followed by one cumulative snapshot is the appropriate next
test if the counter continues increasing.

## Opt-in runtime diagnostics

The `runtime-diagnostics` Cargo feature instruments the complete path for IMU,
magnetometer, barometer, pitot, range, GNSS, battery, RC, scheduler, telemetry, and VCP.
It counts acquisition, errors, handoff replacement or queue pressure, core consumption,
processing, telemetry replacement, telemetry send, timing, and transport acceptance.

The detailed operator tutorial is
[Pixracer Pro runtime sensor diagnostics](../docs/pixracerpro-runtime-diagnostics.md).
The core commands are:

```bash
# Optimized release firmware with diagnostic counters
cargo xtask flash-board pixracerpro --vcp --runtime-diagnostics

# Save before/after snapshots and wrap-safe deltas
python3 tools/capture_runtime_diagnostics.py --duration 60 \
  --output runtime_diagnostics_runs/run.json

# Reanalyze the same data in multiple ways
python3 tools/analyze_runtime_diagnostics.py summary runtime_diagnostics_runs/run.json
python3 tools/analyze_runtime_diagnostics.py stats runtime_diagnostics_runs/*.json
python3 tools/analyze_runtime_diagnostics.py compare before.json after.json
python3 tools/analyze_runtime_diagnostics.py export-csv \
  runtime_diagnostics_runs/*.json --output sensor_runs.csv

# Normal release firmware: no diagnostic feature or counters
cargo xtask flash-board pixracerpro --vcp
```

Each JSON file retains `before`, `after`, wrap-safe counter `deltas`, and boot-wide gauge
values in `observed`. It is therefore suitable for custom `jq`, Python/pandas, R, MATLAB,
or spreadsheet analysis rather than only the bundled report commands.

SWD counter reads halt the MCU briefly. This can create a recognizable boundary gap and
pollute a boot-wide maximum. Use deltas for sustained loss, take sufficiently long runs,
and corroborate rates without the probe using commands such as
`ros2 topic hz /magnetometer`.

Diagnostic captures made during development were stored in the Git-ignored
`runtime_diagnostics_runs/` directory, including:

```text
async_adc_full_pipeline.json
rc_fifo_full_pipeline.json
rc_fifo_controller_connected.json
mag_single_conversion_clean.json
```

Those files are local evidence, not durable repository artifacts, unless deliberately
copied elsewhere.

The flash command now builds the embedded binary with Cargo's `--release` profile,
downloads it with verification, resets the board, and detaches the debug probe. It does
not leave a live `probe-rs run` session holding the MCU. `--vcp` selects the VCP firmware;
it does not enable diagnostics. The diagnostic feature must be requested explicitly.

To verify that a normal release contains no instrumentation symbols:

```bash
arm-none-eabi-nm target/thumbv7em-none-eabihf/release/veloxity | \
  rg VELOXITY_DIAG
```

No output is expected.

## Verification completed during the work

The combined work was checked with:

- 265 `veloxity_core` tests.
- 17 mixer tests.
- 3 `xtask` tests.
- 4 Python diagnostic-tool tests.
- Pixracer Pro normal and diagnostic target builds.
- Nucleo target compilation to ensure shared APIs remained compatible.
- `git diff --check`.
- A normal-release symbol check confirming no `VELOXITY_DIAG` symbols.
- Live Pixracer Pro captures for complete sensor/telemetry flow, async ADC, RC FIFO with
  the controller disconnected and connected, and IST8308 single conversion.

## Open risks and follow-up checklist

1. Commit and push the current working-tree architecture and diagnostic changes. The
   remote current branch does not yet contain everything described after `b82fd21`.
2. Repeat a long magnetometer run without debugger interruptions. Determine whether the
   single DRDY miss was an SWD boundary artifact or a real conversion-timing margin.
3. Add retry/backoff to magnetometer initialization. The current task can return
   permanently after an initialization failure; one later RC-focused diagnostic run
   consequently had no magnetometer data after startup.
4. Exercise pitot, range, and PPS with their actual hardware. Their instrumentation is
   present, but the cited captures did not provide meaningful active-device coverage.
5. Retest VCP pipe headroom whenever new high-rate or large telemetry streams are added.
   The boot-wide minimum-free gauge has reached zero even though measured frame rejection
   remained zero.
6. Treat the eight-frame RC FIFO as measured capacity, not a proof for every future
   configuration. Nonzero full waits or upstream UART errors require reevaluation.
7. For every new sensor, record acquisition rate, acquisition timestamp behavior, CPU
   time, asynchronous wait time, task priority, bus sharing, handoff semantics, maximum
   queue/replacement behavior, core processing count, telemetry count, and VCP acceptance.
8. Preserve the separation between physical sensor-to-board axes and configurable
   board-to-aircraft mounting. Do not restore the C BMI088 X/Y negation merely because
   C contains it.
9. Keep `rosflight_io` unchanged. All interoperability work belongs in Veloxity and is
   tested against the existing ROSflight node behavior.

## Final state in plain language

Across this branch chain, Veloxity learned to read and write the same SD parameter image
as current ROSflight C, report compatible firmware identity, use C's GNSS altitude,
match the relevant sensor units and magnetometer conventions, express IMU installation
in mounting parameters instead of a misleading driver negation, block arming for missing
IMU calibration, reproduce calibration timing and messages, read the Pixracer battery,
send actual fresh samples instead of duplicates, and use available processor slack to
service all due streams.

The later diagnostic work then followed every sample from acquisition through telemetry
and transport. It showed and fixed a lossy one-slot RC handoff, removed blocking battery
ADC CPU use, and traced the remaining 96–97 Hz magnetometer rate to asynchronous
continuous-conversion polling rather than CPU or USB starvation. The diagnostic build
and saved-data tools remain available for proving the same properties when another
sensor or telemetry stream is added.
