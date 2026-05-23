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

## Current Sensor Rates

The GY-91-style module is connected over SPI1:

- MPU accel/gyro: 500 Hz board gate, using a 2.0 ms minimum interval;
- BMP280 pressure/temperature: 50 Hz board gate, using a 20 ms minimum interval;
- magnetometer: absent for this tested module target.

The MPU hardware sample divider is left at 1 kHz. Voloxide currently polls the module because the
visible module header does not expose a data-ready interrupt pin.

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

### UART, 500 Hz IMU Gate

Command:

```bash
python3 tools/mavlink_tester.py --transport uart --device /dev/ttyACM0 --baud 921600 --samples 10000 --duration-s 20 --warmup-s 1 --show 8 --diagnostics
```

Result:

- IMU frames: 9460
- host interval: average 2.008 ms, p99 2.266 ms, max 2.689 ms
- board timestamp interval: average 2.008 ms, p99 2.017 ms, max 2.085 ms
- IMU telemetry rate: 497.9 Hz
- barometer telemetry rate: 24.9 Hz
- status telemetry rate: 10.0 Hz
- firmware loop time: min 168 us, average 222.7 us, p99 264 us, max 268 us
- parser invalid CRC count: 1

Conclusion: the wired UART build can sustain the 500 Hz IMU stream in release mode with comfortable
control-loop margin.

### UART, 500 Hz IMU Gate, SysTick 4 kHz Service Scheduler

The Pico board now uses SysTick to pace core0 service work. The interrupt does not run Voloxide
itself; it only increments a bounded pending-tick counter. `world.run_once()` still runs in thread
mode with single ownership of `World`.

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

### Wi-Fi, 500 Hz IMU Gate, 200 Hz Telemetry Target

Command:

```bash
python3 tools/mavlink_tester.py --transport wifi --board 192.168.1.192 --samples 10000 --duration-s 20 --warmup-s 3 --show 12 --diagnostics
```

Result:

- IMU frames: 2763
- host interval: average 6.154 ms, p99 13.407 ms, max 19.676 ms
- board timestamp interval: average 6.152 ms, p99 6.571 ms, max 13.035 ms
- IMU telemetry rate: 162.6 Hz
- barometer telemetry rate: 23.8 Hz
- status telemetry rate: 9.9 Hz
- firmware loop time: min 253 us, average 426.3 us, p99 937 us, max 1009 us
- parser invalid CRC count: 5

Conclusion: Wi-Fi telemetry throttling reduces pressure, but the CYW43 path still adds jitter to the
RP2350 system. This is acceptable for high-level companion commands with firmware-side stabilization
and timeouts, but it is not a deterministic inner-loop control link.

### Wi-Fi, 500 Hz IMU Gate, 200 Hz Telemetry Target, SysTick 4 kHz Service Scheduler

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

The Pico firmware uses a board-only SysTick scheduler on core0:

- SysTick fires at 4 kHz.
- The SysTick exception increments a bounded pending-tick counter.
- `world.run_once()` runs in thread mode, not interrupt context.
- `World` remains single-owner and synchronous.
- The board driver still gates IMU samples at 500 Hz and barometer samples at 50 Hz.
- The Wi-Fi build continues to run Embassy on core1 for CYW43 and UDP tasks.

This is deliberately not an Embassy interrupt executor migration. Embassy interrupt executors are
available, and the RP-aware path should use `embassy-rp`'s `executor-interrupt` feature with a
software IRQ such as `SWI_IRQ_0`. That is a larger follow-up because the `World` owner would need to
move into a `'static` task and executor/pender ownership must be kept compatible with core1 Wi-Fi.

## Practical Assessment

The 500 Hz wired result shows the RP2350 and current Voloxide hot path are not the limiting factor for
the wired build. The Wi-Fi result shows the remaining issue is specific to the CYW43/UDP/multicore
path, not the SPI sensor gate alone.

For flight architecture, keep the RP2350 responsible for stabilization, attitude/rate loops, motor
outputs, failsafe behavior, and command timeout handling. Use Wi-Fi MAVLink for companion commands,
parameters, telemetry, and mission-level traffic only.
