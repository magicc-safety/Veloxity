# Pico 2 W MAVLink Transport Bring-Up

This guide shows the terminal commands needed to build, flash, and test the Pico 2 W MAVLink
transport modes.

The default firmware uses a normal UART MAVLink link on GPIO0/GPIO1 at 921600 baud. The Wi-Fi
firmware is opt-in at build time and uses the Pico 2 W CYW43439 radio as a UDP MAVLink bridge.
ROSflight does not select between these modes over MAVLink; the selected firmware image decides.

`VOLOXIDE_WIFI_SSID` is the infrastructure Wi-Fi network that the Pico joins, such as a router or
lab access point. It is not a Pico-hosted access point.

The Pico 2 W target selects the telemetry profile at build time:

- UART firmware uses the bounded high-rate profile and streams every accepted IMU sample.
- Wi-Fi firmware uses a Pico-board throttled profile so the flight side can continue sampling faster
  than the CYW43 UDP link publishes.

Current board-side rates:

- UART `SMALL_IMU`: every accepted IMU sample, currently about 500 Hz in release firmware
- Wi-Fi `SMALL_IMU`: requested at 500 Hz, measured about 434 Hz in the current station-mode test
- Wi-Fi `ATTITUDE_QUATERNION` and RC raw: requested at 50 Hz
- Wi-Fi baro, mag, range, differential pressure: requested at 25 Hz
- Wi-Fi `OUTPUT_RAW`: disabled in the current board profile
- battery: requested at 10 Hz
- GNSS: up to 10 Hz

Core defaults remain upstream-like for other boards. MAVLink command, parameter, heartbeat,
status, timesync, statustext, version, and hard-error replies are tagged as high-priority TX.
Sensor telemetry is low-priority TX. The numeric priority type supports arbitrary values; the
named constants are convenience bands only. Wi-Fi RX has the same queue structure, but current
inbound ROSflight traffic is queued at the normal priority level.

## 1. Start In The Voloxide Repo

```bash
cd /home/skink/projects/ROSflight/.distrobox-home/ROSflight/Voloxide
```

## 2. Check The Probe Can See The Pico

```bash
probe-rs list
probe-rs info --chip RP235x
```

If either command cannot see the debug probe, check USB permissions and cabling before continuing.

## 3. Build, Flash, And Verify UART MAVLink Firmware

This is the default mode. It uses one RP2350 core and expects the companion computer MAVLink link
on UART0:

- Pico GPIO0: UART TX from Pico to companion RX
- Pico GPIO1: UART RX from companion TX to Pico
- Pico GND: common ground with companion
- Baud: 921600

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release
probe-rs download --chip RP235x target/thumbv8m.main-none-eabihf/release/voloxide
probe-rs verify --chip RP235x target/thumbv8m.main-none-eabihf/release/voloxide
probe-rs reset --chip RP235x
```

At this point ROSflight-compatible MAVLink should be available on the UART pins. Do not expect Wi-Fi
UDP traffic from this image.

## 4. Build, Flash, And Verify Wi-Fi MAVLink Firmware

Set the Wi-Fi credentials for the router/access point that both the host and Pico can reach:

```bash
export VOLOXIDE_WIFI_SSID='YOUR_ROUTER_WIFI_NAME'
export VOLOXIDE_WIFI_PASSWORD='YOUR_ROUTER_WIFI_PASSWORD'
```

Build the Wi-Fi feature image:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release --features wifi
probe-rs download --chip RP235x target/thumbv8m.main-none-eabihf/release/voloxide
probe-rs verify --chip RP235x target/thumbv8m.main-none-eabihf/release/voloxide
probe-rs reset --chip RP235x
```

This image uses:

- core 0: Voloxide flight-control world
- core 1: CYW43 Wi-Fi and UDP MAVLink bridge
- UDP port: 14550
- Wi-Fi power management: disabled for lower latency

## 5. Find Or Confirm The Pico IP Address

If you know the assigned address, use it directly. You can also check your router's DHCP leases.
The MAVLink tester requires the assigned address for Wi-Fi tests:

```bash
python3 tools/mavlink_tester.py --transport wifi --board 192.168.1.192 --duration-s 8
```

Use the IP address that matches your current Pico DHCP lease.

## 6. Run A UDP Latency Smoke Test

Replace `192.168.1.192` with the Pico's current IP address:

```bash
python3 tools/udp_latency_test.py 192.168.1.192 --count 200 --rate-hz 100 --timeout-ms 250 --payload-bytes 32
```

A healthy station-mode run should receive most packets and report RTT statistics. RTT is
round-trip time, so one-way delivery is roughly half the RTT if the path is symmetric.

For a longer benchmark:

```bash
python3 tools/udp_latency_test.py 192.168.1.192 --count 2000 --rate-hz 100 --timeout-ms 250 --payload-bytes 32
```

## 7. Decode MAVLink Sensor Telemetry

Wi-Fi UDP:

```bash
python3 tools/mavlink_tester.py --transport wifi --board 192.168.1.192 --samples 10000 --duration-s 20 --warmup-s 3 --show 12 --diagnostics
```

Wired UART:

```bash
python3 tools/mavlink_tester.py --transport uart --device /dev/ttyACM0 --baud 921600 --samples 10000 --duration-s 20 --warmup-s 1 --show 8 --diagnostics
```

The tester validates MAVLink v1 checksums, decodes `SMALL_IMU` and `SMALL_BARO`, and reports host
receive intervals plus board timestamp intervals where the message carries a board timestamp.

Release-mode results from the current RP2350 branch:

| Build | IMU telemetry | Board timestamp p99 | Firmware loop p99 | Notes |
| --- | ---: | ---: | ---: | --- |
| UART DMA, 500 Hz gate, fixed 300 MHz | 488.7 Hz | 2.071 ms | 156 us | Physical UART TX/RX is outside the measured world pass. |
| UART DMA, 400 Hz gate, fixed 300 MHz | 393.6 Hz | 2.578 ms | 170 us | Wired path sustains the 400 Hz target. |
| Wi-Fi, 400 Hz gate, batched UDP, fixed 300 MHz | 346.9 Hz | 5.102 ms | 293 us | Improved queue policy, zero CRC errors, but still below target. |
| Wi-Fi, 500 Hz gate, batched UDP, fixed 300 MHz | 409.8 Hz | 4.114 ms | 306 us | Clears 400 Hz by using the better-aligned 2 ms cadence. |

The Wi-Fi number remains lower and more jittery than the wired path. ROSflight should not rely on the
Wi-Fi path for deterministic sub-10 ms control. The RP2350 firmware must own stabilization, failsafe
behavior, and command timeout handling.

Current firmware no longer uses SysTick to grant world-loop passes. Core0 runs `World` continuously;
MAVLink writes enqueue into board-local queues. In UART builds, separate Embassy tasks move physical
bytes with UART DMA, and `serial_flush()` is no-op in the measured world pass. GY-91 SPI sampling is
produced by board service before `update_sensor_bus()` drains pending samples. The Wi-Fi bridge on
core1 batches multiple mailbox reads into one UDP datagram and yields between service passes so
CYW43 work does not dominate core0 timing.

## 8. Sanity Checks Before Flight Testing

Run host tests:

```bash
cargo xtask test-host
```

Run the Pico 2 W board check:

```bash
cargo xtask check-board pico2w
```

Build both firmware variants before changing hardware:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --features wifi
```

For real vehicle testing, keep RC fallback connected directly to the Pico and test Wi-Fi loss,
RC override, RC loss, and disarm behavior before trusting offboard commands.
