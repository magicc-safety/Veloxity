# Pico 2 W MAVLink Transport Bring-Up

This guide shows the terminal commands needed to build, flash, and test the Pico 2 W MAVLink
transport modes.

The default firmware uses a normal UART MAVLink link on GPIO0/GPIO1 at 921600 baud. The Wi-Fi
firmware is opt-in at build time and uses the Pico 2 W CYW43439 radio as a UDP MAVLink bridge.
ROSflight does not select between these modes over MAVLink; the selected firmware image decides.

`VOLOXIDE_WIFI_SSID` is the infrastructure Wi-Fi network that the Pico joins, such as a router or
lab access point. It is not a Pico-hosted access point.

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
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide
probe-rs download --chip RP235x target/thumbv8m.main-none-eabihf/debug/voloxide
probe-rs verify --chip RP235x target/thumbv8m.main-none-eabihf/debug/voloxide
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
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --features wifi
probe-rs download --chip RP235x target/thumbv8m.main-none-eabihf/debug/voloxide
probe-rs verify --chip RP235x target/thumbv8m.main-none-eabihf/debug/voloxide
probe-rs reset --chip RP235x
```

This image uses:

- core 0: Voloxide flight-control world
- core 1: CYW43 Wi-Fi and UDP MAVLink bridge
- UDP port: 14550
- Wi-Fi power management: disabled for lower latency

## 5. Find Or Confirm The Pico IP Address

If you know the assigned address, use it directly. If not, run the UDP bridge without an address and
wait for the firmware discovery beacon:

```bash
python3 tools/udp_mavlink_bridge.py
```

When it prints `learned board=...`, use that IP address for tests. You can also check your router's
DHCP leases.

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

## 7. Run The UDP MAVLink Bridge

With a known board IP:

```bash
python3 tools/udp_mavlink_bridge.py 192.168.1.192
```

Or auto-learn from the firmware beacon:

```bash
python3 tools/udp_mavlink_bridge.py
```

The bridge is intentionally minimal. It is useful for bring-up, discovery, and seeing raw bytes.
ROSflight-side tools should treat the firmware as normal MAVLink once the selected transport is up.

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
