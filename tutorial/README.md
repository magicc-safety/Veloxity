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

The ROScopter waypoint tutorial builds on that bridge and documents the validated one-command
script:

```bash
Voloxide/scripts/run_voloxide_waypoint_demo.zsh
```

The script resets Voloxide's quadrotor parameter store, loads the ROSflight multirotor defaults,
calibrates, starts ROScopter, loads the waypoint mission, and releases RC override.

The ROSplane fixed-wing tutorial keeps the same firmware-boundary replacement but uses the
ROSflight fixed-wing standalone simulator and the ROSplane autonomy stack. The validated visual path
is the manual VimFly handoff script:

```bash
Voloxide/scripts/run_voloxide_rosplane_tutorial_demo.zsh
```

It starts fixed-wing SIL and RViz, pauses for manual VimFly takeoff, then starts ROSplane, loads the
mission, and asks the operator to release RC override with VimFly.
