# Rust SIL Board Shim

## Purpose

RustFlight needs to replace the upstream ROSflight `sil_board` process during simulator testing
without modifying ROSflight packages. The replacement must look like the same ROS graph participant
to `rosflight_sim`, `rosflight_sil_manager`, and `rosflight_io`, while running the RustFlight
firmware core instead of the C++ ROSFlight firmware.

The chosen boundary is a small C++ `rclcpp` shim:

```text
rosflight_sim standalone multirotor
        |
        | ROS 2 topics/services over rmw_zenoh_cpp
        v
rust_sil_board_shim, node name rust_sil_board
        |
        | C ABI FFI boundary
        v
RustFlight firmware core
        |
        | UDP MAVLink
        v
unmodified rosflight_io
```

## Why A C++ Shim

ROS 2 Rust client libraries are not first-class supported APIs for Jazzy in the same way as
`rclcpp` and `rclpy`. The shim keeps the ROS graph boundary on supported `rclcpp` APIs while keeping
firmware behavior in Rust.

This avoids depending on private `rmw_zenoh_cpp` wire details. Zenoh remains the ROS middleware
backend selected by `RMW_IMPLEMENTATION=rmw_zenoh_cpp`; it is not treated as an application protocol
that RustFlight must reverse engineer.

## Package Location

The shim package lives in this repository:

```bash
ros2/rust_sil_board_shim
```

It is not part of the external ROSflight workspace. Build it as a local overlay after sourcing the
installed ROSflight workspace.

## ROS Contract

The shim currently provides:

- node name: `rust_sil_board`
- service: `sil_board/run`
- publisher: `sim/pwm_output`
- subscriptions:
  - `sim/sensors/imu/data`
  - `sim/sensors/imu/temperature`
  - `sim/sensors/mag`
  - `sim/sensors/baro`
  - `sim/sensors/gnss`
  - `sim/sensors/diff_pressure`
  - `sim/sensors/range`
  - `sim/sensors/battery`

The service name intentionally stays `sil_board/run` because `rosflight_sil_manager` calls that
service. The node name can be `rust_sil_board` because service discovery depends on the service
name, not the executable name.

## Current State

The shim compiles, starts, and links against the RustFlight `sim` crate as a Rust `staticlib`.

Its `sil_board/run` service handler now:

- converts the latest ROS sensor messages into plain C FFI structs
- timestamps the RustFlight sensor snapshot with monotonic firmware-clock time
- passes those snapshots into RustFlight
- runs one RustFlight firmware iteration
- reads back RustFlight PWM outputs
- publishes those outputs on `sim/pwm_output`

The Rust side owns the UDP MAVLink socket with default bind `127.0.0.1:14525` and default remote
`127.0.0.1:14520`, matching the unmodified `rosflight_io` SIL UDP defaults.

This has been build- and smoke-tested. A bounded ROS 2 service call through `rmw_zenoh_cpp`
returned:

```text
success=True, message='RustFlight SIL iteration completed'
```

The full standalone multirotor launch has also been validated with `rmw_zenohd`, `rosflight_io`,
`rosflight_sil_manager`, standalone sensors/dynamics/forces, RC, and `rust_sil_board` running
together. Observed validation signals:

- `rosflight_io` connected over UDP to `localhost:14525` from `localhost:14520`.
- `rosflight_io` received RustFlight heartbeat and reported connected.
- `rosflight_io` received all parameters from RustFlight.
- `/status` was published by `rosflight_io` and consumed by `standalone_sensors`.
- `/sim/sensors/imu/data` was published by the standalone simulator.
- `/sim/pwm_output` was published by `rust_sil_board` and consumed by
  `multirotor_forces_and_moments`.

The first full run exposed repeated `Time going backwards` autopilot errors because the shim was
forwarding ROS wall-clock sensor timestamps into RustFlight. The shim now passes monotonic FCU-time
timestamps for FFI sensor snapshots; the repeated time-backwards errors did not reappear in the
second validation run.

Remaining expected runtime warnings:

- `ROSflight version does not match firmware version`, because the firmware reports
  `RustFlight Alpha 0.1`.
- initial `Uncalibrated IMU` and short RC lost/recovered messages during startup.

## Build

```bash
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/rustflight_setup/workspace/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

colcon --log-base target/ros2/log build \
  --base-paths ros2/rust_sil_board_shim \
  --build-base target/ros2/build \
  --install-base target/ros2/install

source target/ros2/install/setup.zsh
```

## Run

Start the ROS 2 Zenoh router:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

In another shell with the same sourced environment:

```bash
ros2 launch rust_sil_board_shim rust_sil_board.launch.py
```

For the current standalone multirotor validation target, launch the local overlay file:

```bash
ros2 launch rust_sil_board_shim multirotor_standalone_rust.launch.py
```

For deterministic RC spoofing, disable the built-in `rosflight_sim` RC node:

```bash
ros2 launch rust_sil_board_shim multirotor_standalone_rust.launch.py use_builtin_rc:=false
```

Do not run the upstream `rosflight_sim` `sil_board` node at the same time. It embeds the C++
ROSFlight firmware and owns the same simulator-board role that this shim replaces.

## Run With Vimfly

Use this path when you want to fly the standalone multirotor interactively from the keyboard using
ROSflight's existing `vimfly` RC frontend.

Terminal 1, start the ROS 2 Zenoh router:

```bash
cd /home/skink/projects/rustflight_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/rustflight_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Terminal 2, launch the Rust-backed standalone multirotor stack with vimfly enabled:

```bash
cd /home/skink/projects/rustflight_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/rustflight_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch rust_sil_board_shim multirotor_standalone_rust.launch.py use_vimfly:=true
```

Terminal 3, load the upstream ROSflight multirotor firmware parameters through unmodified
`rosflight_io`:

```bash
cd /home/skink/projects/rustflight_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/rustflight_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile \
  "{filename: /home/skink/projects/rustflight_setup/workspace/install/rosflight_sim/share/rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml}"
ros2 service call /calibrate_imu std_srvs/srv/Trigger
ros2 service call /param_write std_srvs/srv/Trigger
```

The parameter file includes the multirotor failsafe, RC mapping, controller, motor, and mixer
parameters expected by the ROSflight standalone multirotor sim. This is preferred over manually
setting individual safety/mixer parameters.

Vimfly opens a small pygame window. Focus that window before pressing keys. The ROSflight vimfly
bindings are:

- `a`: increase throttle
- `s`: decrease throttle
- `h`: roll left
- `l`: roll right
- `j`: pitch backward
- `k`: pitch forward
- `d`: yaw counterclockwise
- `f`: yaw clockwise
- `t`: toggle arm
- `r`: toggle RC override

Useful inspection commands:

```bash
ros2 topic echo /status
ros2 topic echo /sim/pwm_output
ros2 topic echo /sim/truth_state
```

## Arming And Motion Validation

The standalone multirotor stack has been validated through arming and non-idle PWM using spoofed RC
input. The validation ran with `use_builtin_rc:=false` so the test publisher was the only
`sim/RC` source.

Required sim setup:

```bash
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile \
  "{filename: /home/skink/projects/rustflight_setup/workspace/install/rosflight_sim/share/rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml}"
ros2 service call /calibrate_imu std_srvs/srv/Trigger
ros2 service call /param_write std_srvs/srv/Trigger
```

The YAML file sets `FAILSAFE_THR=0.0` and the multirotor mixer/controller/motor parameters. The
IMU calibration service is the intended path for producing nonzero IMU bias params and clearing the
uncalibrated-IMU gate. Earlier validation used `ACC_X_BIAS=0.01` as a temporary shortcut; keep that
only as a debugging fallback if the RustFlight calibration command path is under investigation.

RC arming input:

```bash
ros2 topic pub -r 50 /sim/RC rosflight_msgs/msg/RCRaw \
  "{values: [1500, 1500, 1000, 2000, 1000, 1000, 1000, 1000]}"
```

This holds throttle low and yaw high long enough to trigger the default stick arming path.

Low positive throttle input:

```bash
ros2 topic pub -r 50 /sim/RC rosflight_msgs/msg/RCRaw \
  "{values: [1500, 1500, 1300, 1500, 1000, 1000, 1000, 1000]}"
```

Observed validation signals:

- `rosflight_io`: `Autopilot RECOVERED ERROR: Uncalibrated IMU`
- `rosflight_io`: `Parameter FAILSAFE_THR has new value 0`
- `rosflight_io`: `Autopilot ARMED`
- `/status`: `armed: true`, `failsafe: false`, `error_code: 0`
- `/sim/pwm_output`: first four channels moved off idle, sampled as `1102, 1100, 1100, 1103`
- `/sim/truth_state`: pose/twist changed while throttle was active

## Directional Acceptance Test Design

The next acceptance test should prove the axis signs, not just that the vehicle moves. Run it with
`use_builtin_rc:=false` so scripted RC commands are the only `sim/RC` source. The scripted commands
use the same channel values as vimfly:

- roll right: channel 0 = `1000`, matching vimfly `l`
- roll left: channel 0 = `2000`, matching vimfly `h`
- pitch forward: channel 1 = `2000`, matching vimfly `k`
- pitch backward: channel 1 = `1000`, matching vimfly `j`
- yaw clockwise: channel 3 = `1000`, matching vimfly `f`
- yaw counterclockwise: channel 3 = `2000`, matching vimfly `d`

Test setup:

```bash
ros2 launch rust_sil_board_shim multirotor_standalone_rust.launch.py use_builtin_rc:=false
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile \
  "{filename: /home/skink/projects/rustflight_setup/workspace/install/rosflight_sim/share/rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml}"
ros2 service call /calibrate_imu std_srvs/srv/Trigger
ros2 topic pub -r 50 /sim/RC rosflight_msgs/msg/RCRaw \
  "{values: [1500, 1500, 1000, 2000, 1000, 1000, 1000, 1000]}"
```

After `/status` reports `armed: true`, climb into a low hover-like throttle band:

```bash
ros2 topic pub -r 50 /sim/RC rosflight_msgs/msg/RCRaw \
  "{values: [1500, 1500, 1450, 1500, 1000, 1000, 1000, 1000]}"
```

For each axis case, hold the command for a short fixed window, sample `/sim/truth_state` before and
after, then return to neutral. The standalone simulator reports NED truth state, so the expected
signs from a level initial heading are:

- pitch forward should increase north position/velocity: `pose.position.x` and/or `twist.linear.x`
  become more positive than the neutral baseline.
- pitch backward should drive `pose.position.x` and/or `twist.linear.x` negative relative to the
  forward case.
- roll right should increase east position/velocity: `pose.position.y` and/or `twist.linear.y`
  become more positive than the neutral baseline.
- roll left should drive `pose.position.y` and/or `twist.linear.y` negative relative to the right
  case.
- yaw clockwise should produce the opposite yaw-rate sign from yaw counterclockwise. If the absolute
  yaw sign is ambiguous in the observation tooling, the minimum pass criterion is that the two yaw
  commands produce opposite-signed `twist.angular.z` responses.

Pass criteria:

- `/status` stays `armed: true`, `failsafe: false`, and `error_code: 0`.
- `/sim/pwm_output` remains non-idle during the maneuver windows.
- The pitch and roll truth-state responses have the expected signs above.
- The two yaw commands produce opposite-signed yaw-rate responses.
- Repeating the same scripted commands against upstream C `sil_board` produces the same sign pattern.
