# RP2350 Timing Notes

This document records the current Pico 2 W timing state on the `Derekbenj/rp2350` branch.

## Current Firmware Split

The default Pico 2 W firmware is UART-only:

- core 0 runs Voloxide;
- MAVLink is on UART0, GPIO0/GPIO1, 921600 baud;
- no Wi-Fi stack is started.

The `wifi` or `wifi-mavlink` feature enables the Wi-Fi transport:

- core 0 runs Voloxide;
- core 1 owns CYW43, DHCP, UDP, and the MAVLink UDP bridge;
- MAVLink UDP listens on port 14550;
- CYW43 power management is disabled.

The core does not know whether the board selected UART or Wi-Fi. The core exposes generic
`TelemetryRates`; the Pico board chooses a profile based on the build feature.

## Current Core0 Execution Model

Core0 now runs `World` continuously:

```rust
loop {
    world.run_once();
}
```

Timers and peripheral code are event producers, not permission to run the world loop. The Pico board
implementation owns one MAVLink endpoint internally: UART in the default build, or the core1 Wi-Fi
mailbox when `wifi`/`wifi-mavlink` is enabled. `voloxide_core` still sees only the `BoardIo` serial
byte contract.

UART transmit is DMA serviced outside the measured world pass. MAVLink writes enqueue bytes into the
board priority queues; a board-local UART TX task drains those queues into fixed-size DMA batches.
UART RX uses a small fixed DMA chunk and pushes bytes into the same logical RX pipe. For UART builds,
`serial_flush()` is effectively no-op, so `tx_flush_us` should stay near zero. The Wi-Fi build keeps
CYW43, DHCP, UDP RX/TX, and the UDP MAVLink bridge on core1. Core1 pushes inbound UDP bytes into the
same logical RX pipe used by the UART build and drains outbound bytes from the same logical TX pipe.

## Current Sensor Rates

The GY-91-style module is connected over SPI1:

- MPU accel/gyro: 500 Hz board gate, using a 2.0 ms minimum interval;
- BMP280 pressure/temperature: 50 Hz board gate, using a 20 ms minimum interval;
- magnetometer: absent for this tested module target.

The MPU hardware sample divider is left at 1 kHz. The visible module header does not expose a
data-ready interrupt pin, so the current Pico board path uses the board clock as the sensor event
source. GY-91 SPI transactions happen in Pico board producer service, and `BoardIo::update_sensor_bus`
only drains pending samples into `SensorBus`.

## Timing Semantics

Pico 2 W firmware initializes RP2350 `clk_sys` at 300 MHz. There are no build-time sysclk selection
features in the Pico board crate now; 300 MHz is the standard clock for both UART and Wi-Fi firmware.

`World::run_once_measured()` reports board-local pass timing:

- `comm_us`: inbound MAVLink parsing and command/parameter event service;
- `sensor_us`: draining pending board samples, sensor processing, health, and log responses;
- `control_us`: estimator, controller, mixer, and PWM writes when a new IMU sample advances time;
- `telemetry_us`: telemetry enqueue plus bounded `serial_flush()` service;
- `had_rx`, `had_sensor`, `had_imu`, and `ran_control`: coarse pass classification.

The `timing-diagnostics` feature emits compact MAVLink `STATUSTEXT` summaries for test runs only.
`PERF` gives the coarse pass buckets above. `PERC` splits control into estimator, controller, mixer,
and PWM output work. `PERS` splits sensor handling into board-sample drain, sensor processing, health,
and log/response work. `PERT` splits RC/output housekeeping, telemetry enqueue, bounded TX flush, and
deferred board service. These values are measured on the board with the board microsecond clock; UART
or Wi-Fi only carries the already-collected summaries to the tester.

Existing ROSflight status `loop_time_us` is still the control pipeline time, preserving wire
compatibility. Idle and RX-only passes can therefore have measured pass time without changing
`loop_time_us`; that status field only advances when the IMU-driven control pipeline runs.
Board deferred producer service, including the current GY-91 SPI sampling hook, runs at the end of
the pass. UART physical TX/RX DMA service runs in separate board tasks and is reported through
board-local diagnostics when `timing-diagnostics` is enabled.

## Release-Mode Measurements

All numbers below were collected from release firmware with:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release
```

or:

```bash
VOLOXIDE_WIFI_SSID=MAGICC VOLOXIDE_WIFI_PASSWORD=magiccwifi \
  cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release --features wifi-mavlink
```

### UART DMA, 500 Hz IMU Gate

Command:

```bash
python3 tools/mavlink_tester.py --transport uart --device /dev/ttyACM0 --baud 921600 --samples 5000 --duration-s 15 --warmup-s 1 --show 8 --diagnostics
```

Result:

- IMU frames: 5000
- host interval: average 2.046 ms, p99 2.461 ms, max 13.602 ms
- board timestamp interval: average 2.046 ms, p99 2.071 ms, max 4.103 ms
- IMU telemetry rate: 488.7 Hz
- barometer telemetry rate: 24.0 Hz
- status telemetry rate: 9.8 Hz
- firmware loop time: min 130 us, average 137.6 us, p99 156 us, max 162 us
- control-class pass average: 359.2 us
- control-class `tx_flush_us`: 1.0 us
- control-class telemetry enqueue: 97.5 us
- parser invalid CRC count: 3

Conclusion: the wired UART build still carries the high-rate IMU stream in release mode. Moving
physical UART TX to DMA removed the previous synchronous flush cost from the world pass; the remaining
telemetry cost is mostly MAVLink production/enqueue work.

### UART DMA, 400 Hz IMU Gate

Command:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release --features 'timing-diagnostics imu-400hz'
python3 tools/mavlink_tester.py --transport uart --device /dev/ttyACM0 --baud 921600 --samples 5000 --duration-s 15 --warmup-s 1 --show 8 --diagnostics
```

Result:

- IMU frames: 5000
- host interval: average 2.540 ms, p99 2.909 ms, max 14.273 ms
- board timestamp interval: average 2.540 ms, p99 2.578 ms, max 5.094 ms
- IMU telemetry rate: 393.6 Hz
- barometer telemetry rate: 24.6 Hz
- status telemetry rate: 9.7 Hz
- firmware loop time: min 139 us, average 146.1 us, p99 170 us, max 170 us
- control-class pass average: 365.5 us
- control-class `tx_flush_us`: 0.0 us
- control-class telemetry enqueue: 97.0 us
- parser invalid CRC count: 5

Conclusion: the UART DMA path sustains the 400 Hz target with the fixed 300 MHz clock.

### Historical UART, 500 Hz IMU Gate, SysTick 4 kHz Service Scheduler

Earlier firmware used SysTick to pace core0 service work. The interrupt did not run Voloxide itself;
it only incremented a bounded pending-tick counter. `world.run_once()` still ran in thread mode with
single ownership of `World`.

The IMU/control gate remains 500 Hz. The scheduler service rate is 4 kHz so synchronous UART TX/RX
and telemetry work can be serviced between accepted IMU samples.

Result:

- IMU frames: 9470
- host interval: average 2.007 ms, p99 2.326 ms, max 2.797 ms
- board timestamp interval: average 2.006 ms, p99 2.249 ms, max 2.257 ms
- IMU telemetry rate: 498.4 Hz
- barometer telemetry rate: 23.5 Hz
- status telemetry rate: 10.0 Hz
- firmware loop time: min 169 us, average 219.7 us, p99 298 us, max 300 us
- parser invalid CRC count: 1

Earlier scheduler trials at 500 Hz, 1 kHz, and 2 kHz service rates did not leave enough service turns
for the synchronous UART transmit path and reduced observed IMU telemetry to 375.6 Hz, 424.5 Hz, and
469.6 Hz respectively. The final 4 kHz service rate recovers the wired 500 Hz stream without changing
the 500 Hz IMU/control gate.

### Wi-Fi, 400 Hz IMU Gate, Batched UDP, 400 Hz Telemetry Target

Command:

```bash
VOLOXIDE_WIFI_SSID=MAGICC VOLOXIDE_WIFI_PASSWORD=magiccwifi \
  cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release --features 'wifi timing-diagnostics imu-400hz'
python3 tools/mavlink_tester.py --transport wifi --board 192.168.1.192 --samples 5000 --duration-s 15 --warmup-s 3 --show 8 --diagnostics
```

Result:

- IMU frames: 4163
- host interval: average 2.883 ms, p99 7.566 ms, max 114.933 ms
- board timestamp interval: average 2.883 ms, p99 5.102 ms, max 5.221 ms
- IMU telemetry rate: 346.9 Hz
- barometer telemetry rate: 24.5 Hz
- status telemetry rate: 10.0 Hz
- firmware loop time: min 171 us, average 237.4 us, p99 293 us, max 361 us
- parser invalid CRC count: 0
- RX byte rate: 18.9 kB/s

Conclusion: 300 MHz improves the measured control math, but the 400 Hz Wi-Fi target does not deliver
400 Hz in this build. After the low-priority frame-ring queue and more aggressive TX service, the
400 Hz run improved from about 325 Hz to about 347 Hz with zero CRC errors.

The earlier 500 Hz Wi-Fi target exceeded 400 Hz because its 2.0 ms telemetry period aligned with the
default 2.0 ms accepted IMU cadence. The 400 Hz target uses a 2.5 ms cadence and currently delivers
roughly every other/third queued sample over UDP under load.

### Wi-Fi, 500 Hz IMU Gate, Batched UDP, 500 Hz Telemetry Target

Command:

```bash
VOLOXIDE_WIFI_SSID=MAGICC VOLOXIDE_WIFI_PASSWORD=magiccwifi \
  cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release --features 'wifi timing-diagnostics'
python3 tools/mavlink_tester.py --transport wifi --board 192.168.1.192 --samples 5000 --duration-s 15 --warmup-s 3 --show 8 --diagnostics
```

Result:

- IMU frames: 4871
- host interval: average 2.440 ms, p99 6.515 ms, max 140.519 ms
- board timestamp interval: average 2.794 ms, p99 4.114 ms, max 1518.751 ms
- IMU telemetry rate: 409.8 Hz
- barometer telemetry rate: 24.6 Hz
- status telemetry rate: 12.8 Hz
- firmware loop time: min 106 us, average 217.6 us, p99 306 us, max 315 us
- parser invalid CRC count: 0
- RX byte rate: 22.1 kB/s

Conclusion: the 500 Hz Wi-Fi telemetry cadence now exceeds 400 Hz in station-mode testing. The 400 Hz
requested cadence still under-delivers, so the limiting factor appears to be cadence/transport
interaction rather than control math time.

### Historical Wi-Fi, 500 Hz IMU Gate, 200 Hz Telemetry Target, SysTick 4 kHz Service Scheduler

Result:

- IMU frames: 2670
- host interval: average 6.369 ms, p99 13.542 ms, max 104.136 ms
- board timestamp interval: average 6.369 ms, p99 6.758 ms, max 12.877 ms
- IMU telemetry rate: 157.0 Hz
- barometer telemetry rate: 23.5 Hz
- status telemetry rate: 9.9 Hz
- firmware loop time: min 219 us, average 429.0 us, p99 838 us, max 1060 us
- parser invalid CRC count: 3

The scheduler change improves Wi-Fi firmware loop p99 compared with the earlier 937 us result, but
it does not fix the CYW43/UDP throughput limit. That limit remains in the board transport path, not
in Voloxide core math.

## RAM Placement Check

The current Pico linker script defines:

```ld
RAM   : ORIGIN = 0x20000000, LENGTH = 512K
SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
```

That means SRAM8/SRAM9 are cleanly named, but each bank is only 4 KiB. They are not good targets for
the full core0 stack or the current 64 KiB core1 stack. They may be useful later for very small hot
state or scratch buffers.

Moving hot code into RAM needs linker support that copies executable code from flash into SRAM at
startup. A naive `#[link_section]` on hot functions is not enough unless the section is included in a
copied RAM region and every called hot function is also resident or inlined. This should be measured
as a separate change.

## Scheduler Check

The Pico firmware no longer gates core0 `World` execution with SysTick. `World` remains single-owner
and synchronous in thread mode, but it is always runnable. Board peripherals produce events into
queues or pending-sample slots, and each world pass drains what is available without performing
unbounded TX work.

This is deliberately not an Embassy interrupt executor migration. Embassy interrupt executors are
available, and the RP-aware path could use `embassy-rp`'s `executor-interrupt` feature with a
software IRQ such as `SWI_IRQ_0`. That remains a larger follow-up because the `World` owner would
need to move into a `'static` task and executor/pender ownership must stay compatible with core1
Wi-Fi.

## Practical Assessment

The 500 Hz wired result shows the RP2350 and current Voloxide hot path are not the limiting factor for
the wired build. The Wi-Fi result shows the remaining issue is specific to the CYW43/UDP/multicore
path, not the SPI sensor gate alone.

For flight architecture, keep the RP2350 responsible for stabilization, attitude/rate loops, motor
outputs, failsafe behavior, and command timeout handling. Use Wi-Fi MAVLink for companion commands,
parameters, telemetry, and mission-level traffic only.
