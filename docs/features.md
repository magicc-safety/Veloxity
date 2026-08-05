# Feature Flags

Cargo features are compile-time switches. They can include optional dependencies, select alternate
hardware behavior, or add diagnostics to a firmware image.

Features belong to individual Cargo packages. Specify the package when enabling a feature:

```bash
cargo build -p pixracerpro --target thumbv7em-none-eabihf --bin veloxity --release \
  --features 'usb-vcp-serial'
```

## Core Features

These features are declared in `crates/veloxity_core/Cargo.toml`. Board packages may expose
matching features that enable the corresponding core feature.

| Feature | Purpose | Normal flight build? |
| --- | --- | --- |
| `scope-timing-pins` | Disables the core's default full-control test-pin pulse so a board can use its test pins for targeted timing instrumentation. The board must provide the physical pin behavior. | No; measurement-only. |
| `pre-control-scope` | Pulses test pin 3 while the IMU sample is read, processed, and checked before the control pipeline runs. | No; measurement-only. |
| `rc-command-scope` | Reserved for RC command/state timing. The current core code does not emit a timing pulse for this feature. | No; currently unimplemented. |
| `control-scope-estimator` | Pulses test pin 3 while the estimator runs. | No; measurement-only. |
| `control-scope-controller` | Pulses test pin 3 while the controller runs. | No; measurement-only. |
| `control-scope-mixer` | Pulses test pin 3 while the mixer runs. | No; measurement-only. |
| `control-scope-pwm` | Pulses test pin 3 while PWM outputs are configured, composed, and written. | No; measurement-only. |

Enable at most one `control-scope-*` feature at a time. The core rejects builds that select more
than one because all four features use the same test pin. These features are useful only on a board
that implements the corresponding test-pin output.

## Pixracer Pro Features

These features are declared in `boards/pixracerpro/Cargo.toml`.

| Feature | Purpose | Normal flight build? |
| --- | --- | --- |
| `usb-vcp-serial` | Uses USB virtual COM port (VCP) instead of the companion-computer UART for MAVLink receive and transmit. | No; optional transport. |
| `scope-timing-pins` | Leaves the Pixracer Pro test pins available for targeted timing instrumentation and disables their built-in indicator pulses. It does not assign a permanent meaning to those pins by itself. | No; measurement-only. |
| `sensor-poll-diagnostics` | Records sensor-poll success and error counters that can be inspected with a debugger, and periodically logs SBUS receiver diagnostics. | No; diagnostic-only. |

The standard Pixracer Pro flash command builds an optimized firmware image with UART MAVLink and no
optional features:

```bash
cargo xtask flash-board pixracerpro
```

Use the matching flash options to enable individual features:

```bash
cargo xtask flash-board pixracerpro --vcp
cargo xtask flash-board pixracerpro --scope-timing-pins
cargo xtask flash-board pixracerpro --sensor-poll-diagnostics
```

The options can be combined when more than one behavior is needed:

```bash
cargo xtask flash-board pixracerpro --vcp --sensor-poll-diagnostics
```

The Nucleo-H753ZI package does not currently declare any board-specific Cargo features.

## Why Features Matter

Feature flags change the compiled firmware image. When reporting a test or timing result, record the
exact feature set, target, optimization mode, and board. A diagnostic build is not identical to the
standard flight build.
