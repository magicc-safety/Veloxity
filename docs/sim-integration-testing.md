# Sim Integration Testing

## Branch Purpose

The ROSflight compatibility migration is complete enough that this branch should now focus on
testing Voloxide as the firmware endpoint inside the ROSflight simulator stack.

The goal is to run the Voloxide `sim` binary against the local ROSflight ROS 2 workspace, connect
it to `rosflight_io`, and verify that data moves through the same firmware-facing contract expected
by ROSflight software-in-the-loop workflows.

## Local Integration Target

The local ROS workspace is outside this repository at:

```bash
/home/skink/projects/voloxide_setup/workspace
```

It contains installed ROSflight packages, including:

- `rosflight_io`
- `rosflight_sim`
- `rosflight_msgs`
- `rosflight_gcs`

The workspace can be sourced with:

```bash
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
```

or, from a bash shell:

```bash
source /home/skink/projects/voloxide_setup/workspace/install/setup.bash
```

## Target Voloxide Sim Shape

Voloxide should stand in for the upstream ROSflight `sil_board` node without modifying
`rosflight_io`, `rosflight_sim`, or any other ROSflight package.

In upstream standalone SIL, `sil_board` is not only a UDP MAVLink endpoint. It also embeds the C++
ROSFlight firmware, subscribes to simulator sensor topics, exposes the `sil_board/run` service, and
publishes actuator output.

For this branch, Voloxide should provide the same observable node behavior while running the Rust
firmware core:

- replacement node named `voloxide_sil_board`
- service behavior compatible with `sil_board/run` so existing ROSflight orchestration can drive it
- UDP MAVLink firmware link compatible with unmodified `rosflight_io`
- sensor ingestion compatible with `rosflight_sim` standalone multirotor
- actuator output publishing compatible with `rosflight_sim` standalone multirotor
- scheduler behavior compatible with `rosflight_sil_manager`

The existing Voloxide sim crate has an outdated Zenoh-facing sensor and actuator backend:

- subscribes to CDR-encoded sensor samples over Zenoh
- consumes IMU, mag, baro, GNSS, and RC inputs
- publishes PWM output over Zenoh
- drives the core `World` scheduler from a `rust/tick` Zenoh topic

The target backend is ROS 2 over Zenoh using `rmw_zenoh_cpp` for Jazzy, so the Rust
`voloxide_sil_board` replacement can consume/publish the same ROS graph data through Zenoh-backed ROS
communication while keeping namespacing and HAL parsing inside Rust.

The MAVLink firmware link remains UDP, which matches the unmodified `rosflight_io` SIL transport:

- Voloxide default bind: `127.0.0.1:14557`
- Voloxide default remote: `127.0.0.1:14520`
- `rosflight_io` default UDP bind: `localhost:14520`
- `rosflight_io` default UDP remote: `localhost:14525`

The Voloxide bind port should be changed to `14525` for direct `rosflight_io` compatibility.

## Immediate Test Objective

Prove an end-to-end data path:

1. Source the ROSflight workspace.
2. Start the ROSflight ROS nodes needed for the simulator and `rosflight_io`.
3. Run the Voloxide replacement sim binary that provides the `voloxide_sil_board` node.
4. Verify that `rosflight_io` receives Voloxide heartbeat/status/parameter telemetry over UDP.
5. Verify that `rosflight_sil_manager` can call the replacement `sil_board/run` service path.

This objective has been validated with the local launch file
`voloxide_sil_board_shim/multirotor_standalone_voloxide.launch.py`. The stack connected through UDP,
`rosflight_io` received heartbeat and all parameters, `/status` and `/sim/pwm_output` were flowing,
and the standalone simulator sensor topics were active.

The arming and low-throttle motion path has also been validated by launching with
`use_builtin_rc:=false`, setting temporary sim-safe params through unmodified `rosflight_io`,
spoofing `sim/RC`, and observing `armed: true`, non-idle PWM, and changing `sim/truth_state`.

The normal startup path should use ROSflight's parameter-file service rather than setting failsafe
parameters by hand:

```bash
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile \
  "{filename: /home/skink/projects/voloxide_setup/workspace/install/rosflight_sim/share/rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml}"
ros2 service call /calibrate_imu std_srvs/srv/Trigger
ros2 service call /param_write std_srvs/srv/Trigger
```

The YAML sets `FAILSAFE_THR=0.0` along with the multirotor RC, controller, motor, and mixer params.
The IMU calibration service is the intended way to populate nonzero IMU bias params. The previous
manual `ACC_X_BIAS=0.01` step was a temporary debugging shortcut, not the preferred operator flow.
The next validation run should use this YAML-plus-calibration path.

The YAML-plus-calibration path has since been validated against both Voloxide and upstream C
`sil_board`. Directional scripted RC tests matched upstream C sign behavior:

- channel 1 high, vimfly `k`, produced negative north velocity.
- channel 1 low, vimfly `j`, produced positive north velocity.
- channel 0 low, vimfly `l`, produced negative east velocity.
- channel 0 high, vimfly `h`, produced positive east velocity.
- channel 3 low/high, vimfly `f`/`d`, produced opposite signed yaw rates.

This means Voloxide currently matches ROSflight's standalone simulator sign behavior. The vimfly
direction labels should be interpreted through the upstream sim behavior rather than assuming
positive NED X/Y deltas.

## Backend Work

The branch does not need Zenoh for the firmware-to-`rosflight_io` MAVLink link. `rosflight_io`
already expects UDP in ROSflight SIL mode.

The branch needs the `sim` binary replaced with a clean Rust replacement for the upstream ROSflight
firmware wrapper:

- UDP MAVLink byte transport compatible with `rosflight_io`
- ROSflight `sim/sensors/*` topic ingestion into Voloxide's `SensorBus`
- Voloxide PWM/output publishing to `sim/pwm_output`
- scheduler tick source compatible with the standalone simulator loop
- `sil_board/run` service compatibility so `rosflight_sil_manager` can drive Voloxide

Zenoh is the intended ROS communication backend for this branch. Use it for ROS graph transport and
Rust-side HAL message ingestion/publication, while leaving the MAVLink link to `rosflight_io` on UDP.

Use a small C++ `rclcpp` shim as the ROS 2 client boundary and let `rmw_zenoh_cpp` provide the
Zenoh transport underneath. Avoid depending on private `rmw_zenoh` key-expression details for ROS
topics or services, especially for `sil_board/run`.

The shim lives in this repository at:

```bash
ros2/voloxide_sil_board_shim
```

See `docs/voloxide-sil-board-shim.md` for the focused shim design and build notes.

It is intentionally a ROS 2 package, not a ROSflight package. Build it as an overlay after sourcing
Jazzy and the local ROSflight workspace. This keeps ROSflight source and install artifacts
unmodified while giving Voloxide an officially supported ROS 2 boundary through `rclcpp`.

The shim owns:

- node name: `voloxide_sil_board`
- service: `sil_board/run`
- publish: `sim/pwm_output`
- subscribe: `sim/sensors/imu/data`
- subscribe: `sim/sensors/imu/temperature`
- subscribe: `sim/sensors/mag`
- subscribe: `sim/sensors/baro`
- subscribe: `sim/sensors/gnss`
- subscribe: `sim/sensors/diff_pressure`
- subscribe: `sim/sensors/range`
- subscribe: `sim/sensors/battery`

The shim links against the Voloxide `sim` crate through a C ABI FFI boundary. Its service handler
passes the latest sensor snapshots into Voloxide, runs one Voloxide firmware iteration, reads
back PWM outputs, and publishes them on `sim/pwm_output`.

The implementation should keep edits scoped to Voloxide code in this repository. ROSflight ROS
nodes may be run from the local workspace for integration testing, but the ROSflight source tree
should remain a reference and runtime dependency rather than an edit target.

## Implementation Decisions

- Do not modify ROSflight packages or launch files.
- Use `rosflight_sim` standalone multirotor for all current testing.
- Name the Rust replacement node `voloxide_sil_board`.
- Replace the current Voloxide `sim` binary rather than layering on the outdated Zenoh syntax.
- Install/use `rmw_zenoh_cpp` for ROS Jazzy, then run ROS nodes with
  `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
- Treat heartbeat/status/parameters plus `sil_board/run` service compatibility as the first
  acceptance target before closed-loop dynamics.

## Verified Container Setup

ROS Jazzy is installed under:

```bash
/opt/ros/jazzy
```

The local ROSflight workspace is sourced after Jazzy:

```bash
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
```

`ros-jazzy-rmw-zenoh-cpp` has been installed in the container. It provides:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Use this environment for ROSflight sim runs:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

The packaged Zenoh config files are at:

```bash
/opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
/opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5
```

## Building The Shim

From the repository root:

```bash
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
colcon --log-base target/ros2/log build --base-paths ros2/voloxide_sil_board_shim --build-base target/ros2/build --install-base target/ros2/install
source target/ros2/install/setup.zsh
```

Run the Zenoh router for ROS 2:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Run the shim:

```bash
ros2 launch voloxide_sil_board_shim voloxide_sil_board.launch.py
```

Do not launch the upstream `rosflight_sim` `sil_board` node at the same time. It embeds the C++
ROSFlight firmware and owns the same `sil_board/run` service role that this shim replaces.
