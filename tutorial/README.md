# Tutorial

This folder contains the current working Voloxide software-in-the-loop path into the ROSflight 2.0
ecosystem.

Start here:

- [1. Connect Voloxide Firmware To ROSflight SIL](voloxide-firmware-bridge.md)
- [2. Run ROScopter Waypoint Following With Voloxide](voloxide-roscopter-waypoints.md)
- [3. Run ROSplane Fixed-Wing Waypoint Following With Voloxide](voloxide-rosplane-waypoints.md)
- [SIL Findings](sil-findings.md)

The first tutorial is intentionally narrow: it replaces only the ROSflight SIL firmware endpoint.
After that, the normal ROSflight 2.0 / ROScopter architecture should behave the same whether the
firmware endpoint is upstream C or Voloxide/Rust.

The waypoint tutorial builds on that bridge and adds the launch order, calibration sequence,
middleware, estimator parameter choices, and path-manager settings that produced the current working
GUI waypoint run.

The fixed-wing tutorial keeps the same firmware-boundary replacement but uses the ROSflight
fixed-wing standalone simulator and the ROSplane autonomy stack.
