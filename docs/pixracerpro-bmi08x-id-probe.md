# Pixracer Pro BMI08x identity probe

Pixracer Pro boards may contain either a BMI085 or BMI088 accelerometer. The two
variants report different chip IDs and support different acceleration ranges, so
the firmware must select the matching range conversion. The standalone
`bmi08x_id_probe` executable reads the accelerometer chip ID over SPI5 without
starting the flight-control firmware or PWM outputs.

The probe source is in
`boards/pixracerpro/src/bin/bmi08x_id_probe.rs`. It is a separate Cargo binary:
the normal `cargo xtask flash-board pixracerpro` path explicitly builds and
flashes only `--bin veloxity`. This Markdown file and the probe executable are
therefore not included in the normal flight-firmware image.

## Connect and identify the debug probe

Connect the SWD debugger and power the Pixracer Pro, then verify that probe-rs
can see it:

```bash
probe-rs list
```

The procedure works with a supported J-Link or ST-Link. The board used to
validate this probe was connected through an ST-Link V2.

## Build the identity probe

From the Veloxity repository root, run:

```bash
cargo build -p pixracerpro \
  --target thumbv7em-none-eabihf \
  --bin bmi08x_id_probe \
  --release \
  --no-default-features \
  --features mcu-h743ii
```

This produces
`target/thumbv7em-none-eabihf/release/bmi08x_id_probe`.

## Flash and read the result

Run the probe under probe-rs so its semihosting message appears in the terminal:

```bash
probe-rs run \
  --chip STM32H743IIKx \
  --protocol swd \
  --speed 4000 \
  target/thumbv7em-none-eabihf/release/bmi08x_id_probe
```

Interpret the result as follows:

| Accelerometer ID | Variant | Supported full-scale ranges |
| --- | --- | --- |
| `0x1E` | BMI088 | +/-3 g, +/-6 g, +/-12 g, or +/-24 g |
| `0x1F` | BMI085 | +/-2 g, +/-4 g, +/-8 g, or +/-16 g |

The tested Pixracer Pro reported:

```text
Pixracer BMI08x probe: ID 0x1F => BMI085
```

The result is also stored in the debugger-visible
`PIXRACER_BMI_ACCEL_ID` atomic. An SPI failure stores `0xFFFFFFFE`; the initial
value before a completed read is `0xFFFFFFFF`.

The requested 4000 kHz SWD speed is an upper request, not a guarantee. probe-rs
may report that the connected debugger negotiated a lower speed; this does not
change the sensor result.

Press Ctrl-C after reading the message.

## Restore the flight firmware

The identity probe replaces the firmware currently in flash and intentionally
does not run Veloxity. Always restore the normal flight firmware before using the
board:

```bash
cargo xtask flash-board pixracerpro
```

That command builds and flashes only the optimized `veloxity` binary, verifies
the programmed image, and resets the Pixracer Pro.
