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

The current Pico firmware uses a synchronous core0 loop:

```rust
loop {
    world.run_once();
}
```

The board driver gates IMU and barometer sampling by timestamp. The Wi-Fi build runs Embassy on
core1 for CYW43 and UDP tasks.

Embassy interrupt executors are available in the dependency stack, and the STM32 platform already
uses `InterruptExecutor` for prioritized async work. The Pico board has not yet enabled
`executor-interrupt` or converted core0 into an interrupt-driven executor. That change should be a
dedicated scheduler refactor:

- enable the proper Embassy executor interrupt feature for Pico;
- use a timer interrupt or real IMU DRDY interrupt when a sensor exposes one;
- keep Wi-Fi work on core1;
- ensure core0 control work cannot block on telemetry TX;
- re-run release timing after the scheduler change.

## Practical Assessment

The 500 Hz wired result shows the RP2350 and current Voloxide hot path are not the limiting factor for
the wired build. The Wi-Fi result shows the remaining issue is specific to the CYW43/UDP/multicore
path, not the SPI sensor gate alone.

For flight architecture, keep the RP2350 responsible for stabilization, attitude/rate loops, motor
outputs, failsafe behavior, and command timeout handling. Use Wi-Fi MAVLink for companion commands,
parameters, telemetry, and mission-level traffic only.
