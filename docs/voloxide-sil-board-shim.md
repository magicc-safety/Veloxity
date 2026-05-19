# Voloxide SIL Board Shim

## Purpose

Voloxide needs to replace the upstream ROSflight `sil_board` process during simulator testing
without modifying ROSflight packages. The replacement must look like the same ROS graph participant
to `rosflight_sim`, `rosflight_sil_manager`, and `rosflight_io`, while running the Voloxide
firmware core instead of the C++ ROSFlight firmware.

The chosen boundary is a small C++ `rclcpp` shim:

```text
rosflight_sim standalone multirotor
        |
        | ROS 2 topics/services over rmw_zenoh_cpp
        v
voloxide_sil_board_shim, node name voloxide_sil_board
        |
        | C ABI FFI boundary
        v
Voloxide firmware core
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
that Voloxide must reverse engineer.

## Package Location

The shim package lives in this repository:

```bash
ros2/voloxide_sil_board_shim
```

It is not part of the external ROSflight workspace. Build it as a local overlay after sourcing the
installed ROSflight workspace.

## ROS Contract

The shim currently provides:

- node name: `voloxide_sil_board`
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
service. The node name can be `voloxide_sil_board` because service discovery depends on the service
name, not the executable name.

## Current State

The shim compiles, starts, and links against the Voloxide `sim` crate as a Rust `staticlib`.
The ROS 2 shim build always invokes `cargo build -p sim --lib` before linking so Rust source
changes are reflected in the installed `voloxide_sil_board` executable.

Its `sil_board/run` service handler now:

- converts the latest ROS sensor messages into plain C FFI structs
- timestamps the Voloxide sensor snapshot with monotonic firmware-clock time
- passes those snapshots into Voloxide
- runs one Voloxide firmware iteration
- reads back Voloxide PWM outputs
- publishes those outputs on `sim/pwm_output`

The Rust side owns the UDP MAVLink socket with default bind `127.0.0.1:14525` and default remote
`127.0.0.1:14520`, matching the unmodified `rosflight_io` SIL UDP defaults.

This has been build- and smoke-tested. A bounded ROS 2 service call through `rmw_zenoh_cpp`
returned:

```text
success=True, message='Voloxide SIL iteration completed'
```

The full standalone multirotor launch has also been validated with `rmw_zenohd`, `rosflight_io`,
`rosflight_sil_manager`, standalone sensors/dynamics/forces, RC, and `voloxide_sil_board` running
together. Observed validation signals:

- `rosflight_io` connected over UDP to `localhost:14525` from `localhost:14520`.
- `rosflight_io` received Voloxide heartbeat and reported connected.
- `rosflight_io` received all parameters from Voloxide.
- `/status` was published by `rosflight_io` and consumed by `standalone_sensors`.
- `/sim/sensors/imu/data` was published by the standalone simulator.
- `/sim/pwm_output` was published by `voloxide_sil_board` and consumed by
  `multirotor_forces_and_moments`.

The first full run exposed repeated `Time going backwards` autopilot errors because the shim was
forwarding ROS wall-clock sensor timestamps into Voloxide. The shim now passes monotonic FCU-time
timestamps for FFI sensor snapshots; the repeated time-backwards errors did not reappear in the
second validation run.

Remaining expected runtime warnings:

- `ROSflight version does not match firmware version`, because the firmware reports
  `Voloxide 0.1`.
- initial `Uncalibrated IMU` and short RC lost/recovered messages during startup.

## Build

```bash
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

colcon --log-base target/ros2/log build \
  --base-paths ros2/voloxide_sil_board_shim \
  --build-base target/ros2/build \
  --install-base target/ros2/install

source target/ros2/install/setup.zsh
```

## Run

Start the ROS 2 Zenoh router:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Some nodes may print an `rmw_zenoh_cpp` warning that they were unable to connect to a Zenoh router
after one attempt. In the current launch order this can happen when a node starts before
`rmw_zenohd` has finished advertising. Treat it as a startup race unless ROS graph traffic is also
missing. In successful runs, cross-process topics and services still flow after the router comes up.

In another shell with the same sourced environment:

```bash
ros2 launch voloxide_sil_board_shim voloxide_sil_board.launch.py
```

For the current standalone multirotor validation target, launch the local overlay file:

```bash
ros2 launch voloxide_sil_board_shim multirotor_standalone_voloxide.launch.py
```

For deterministic RC spoofing, disable the built-in `rosflight_sim` RC node:

```bash
ros2 launch voloxide_sil_board_shim multirotor_standalone_voloxide.launch.py use_builtin_rc:=false
```

Do not run the upstream `rosflight_sim` `sil_board` node at the same time. It embeds the C++
ROSFlight firmware and owns the same simulator-board role that this shim replaces.

## Run With Vimfly

Use this path when you want to fly the standalone multirotor interactively from the keyboard using
ROSflight's existing `vimfly` RC frontend.

Terminal 1, start the ROS 2 Zenoh router:

```bash
cd /home/skink/projects/voloxide_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Terminal 2, launch the Rust-backed standalone multirotor stack with vimfly enabled:

```bash
cd /home/skink/projects/voloxide_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch voloxide_sil_board_shim multirotor_standalone_voloxide.launch.py use_vimfly:=true
```

Terminal 3, load the upstream ROSflight multirotor firmware parameters through unmodified
`rosflight_io`:

```bash
cd /home/skink/projects/voloxide_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile \
  "{filename: /home/skink/projects/voloxide_setup/workspace/install/rosflight_sim/share/rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml}"
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
  "{filename: /home/skink/projects/voloxide_setup/workspace/install/rosflight_sim/share/rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml}"
ros2 service call /calibrate_imu std_srvs/srv/Trigger
ros2 service call /param_write std_srvs/srv/Trigger
```

The YAML file sets `FAILSAFE_THR=0.0` and the multirotor mixer/controller/motor parameters. The
IMU calibration service is the intended path for producing nonzero IMU bias params and clearing the
uncalibrated-IMU gate. Earlier validation used `ACC_X_BIAS=0.01` as a temporary shortcut; keep that
only as a debugging fallback if the Voloxide calibration command path is under investigation.

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
`use_builtin_rc:=false` so scripted RC commands are the only `sim/RC` source. Keep RViz enabled
during launch-based testing unless the run is explicitly headless; this lets a developer watch the
vehicle while the script drives ROS services and RC topics. The scripted commands use the same
channel values as vimfly:

- roll right: channel 0 = `1000`, matching vimfly `l`
- roll left: channel 0 = `2000`, matching vimfly `h`
- pitch forward: channel 1 = `2000`, matching vimfly `k`
- pitch backward: channel 1 = `1000`, matching vimfly `j`
- yaw clockwise: channel 3 = `1000`, matching vimfly `f`
- yaw counterclockwise: channel 3 = `2000`, matching vimfly `d`

Test setup:

```bash
ros2 launch voloxide_sil_board_shim multirotor_standalone_voloxide.launch.py use_builtin_rc:=false
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile \
  "{filename: /home/skink/projects/voloxide_setup/workspace/install/rosflight_sim/share/rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml}"
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
after, then return to neutral. The standalone simulator reports NED truth state, but the first
acceptance criterion is parity with upstream C `sil_board`, not an independent reinterpretation of
the vimfly labels.

The same scripted test has been run against Voloxide and upstream C `sil_board`. Both produced
the same sign pattern within noise:

- `pitch_forward_ch1_2000`: negative north velocity delta.
- `pitch_backward_ch1_1000`: positive north velocity delta.
- `roll_right_ch0_1000`: negative east velocity delta.
- `roll_left_ch0_2000`: positive east velocity delta.
- `yaw_cw_ch3_1000`: negative yaw-rate delta.
- `yaw_ccw_ch3_2000`: positive yaw-rate delta.

Pass criteria:

- `/status` stays `armed: true`, `failsafe: false`, and `error_code: 0`.
- `/sim/pwm_output` remains non-idle during the maneuver windows.
- The pitch, roll, and yaw truth-state responses match the upstream C `sil_board` sign pattern.
- The paired axis commands produce opposite-signed responses.

Automated command:

```bash
cd /home/skink/projects/voloxide_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
python3 scripts/sim_directional_acceptance.py --baseline rust --use-rviz
```

## ROScopter Waypoint Acceptance Test Design

ROScopter provides the waypoint-following layer above ROSflight. The ROSflight tutorial launches the
standalone multirotor sim first, then launches `roscopter_sim sim.launch.py`, then loads or publishes
waypoints through the `/path_planner` services. The Rust-backed version follows the same contract
without changing ROSflight or ROScopter packages:

```text
rosflight_sim standalone multirotor + voloxide_sil_board
        |
        | /status, /imu/data, /gnss, /baro, /command, /sim/truth_state
        v
roscopter estimator/controller/path_manager/path_planner
```

The first waypoint smoke test uses service-published NED waypoints instead of editing the upstream
mission YAML. It starts RViz, starts the Rust-backed standalone stack, loads the standard multirotor
firmware parameters, calibrates, arms with spoofed RC, starts ROScopter, publishes a small NED target,
and checks that the ROScopter command chain reaches Voloxide and produces non-idle PWM.

Automated command:

```bash
cd /home/skink/projects/voloxide_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
python3 scripts/sim_roscopter_waypoint_acceptance.py --use-rviz
```

The script launches:

- `rmw_zenoh_cpp/rmw_zenohd`
- `voloxide_sil_board_shim multirotor_standalone_voloxide.launch.py use_builtin_rc:=false use_rviz:=true`
- ROScopter controller/path nodes, `roscopter_gcs rviz_waypoint_publisher`, and
  `roscopter_sim sim_state_transcriber`

Acceptance criteria:

- `rosflight_io` reports armed and no failsafe.
- Voloxide reports offboard control active and no RC override bits.
- ROScopter publishes `/waypoints`, `/trajectory_command`, `/high_level_command`, and `/command`
  after a waypoint is added.
- The commanded target is NED `[4.0, 0.0, -3.0]`.
- `/command` carries a nonzero thrust command and Voloxide publishes non-idle PWM.
- The NED distance to the target reaches `--waypoint-tolerance` during the test window.

Latest observed result from the current branch:

```text
target_ned=(4.0, 0.0, -3.0)
distance_start=4.502 distance_min=4.502 distance_end=60.242 tolerance=3.000
max_command_thrust=44.900
status: armed=True failsafe=False offboard=True control_mode=0 rc_override=0 error_code=0
roscopter_counts: waypoints=1 trajectory=381 high_level=413 command=513
last_high_level: mode=10 valid=True cmd=(0.281,-1.311,0.932,362.407)
max_pwm_delta=1000
last_command: mode=0 ignore=0 u0_3=[0.0, 0.0, -44.9, 0.0]
ROSCOPTER WAYPOINT RESPONSE FAILED
```

This proves only that the ROScopter waypoint command chain reaches Voloxide as offboard control,
clears RC override, and drives actuator outputs. It does not prove waypoint following, because the
vehicle did not reach the waypoint and moved farther from the target. The acceptance script must fail
this case.

The earlier waypoint test showed `/command` activity but no meaningful motion:

```text
status: armed=True failsafe=False offboard=False control_mode=1 rc_override=960 error_code=0
distance_start=5.000 distance_end=4.997
last_command: mode=2 ignore=0 u0_3=[0.0, 0.0, 0.85, 0.0]
```

Root cause: Voloxide's handwritten MAVLink byte parser used CRC extra `190` for
`OFFBOARD_CONTROL` message ID `180`, while the ROSflight C MAVLink headers and Rust generated
dialect both use CRC extra `90`. The bad CRC table entry caused incoming offboard frames from
unmodified `rosflight_io` to be rejected before they reached `CommManager`. The parser now uses CRC
extra `90`, and `cargo test -p voloxide_core offboard_control_wire_frame_passes_crc_and_decodes`
covers this wire-frame path.

## ROScopter Tutorial Mission Test

The same script can run the first four NED waypoints from the ROScopter tutorial mission. The local
mission YAML is installed with the shim package at:

```bash
ros2/voloxide_sil_board_shim/config/roscopter_four_waypoints.yaml
```

The waypoints are:

- `GOTO [0.0, 0.0, -10.0]`, speed `4.0`
- `GOTO [20.0, 0.0, -10.0]`, speed `4.0`
- `HOLD [20.0, -20.0, -20.0]`, speed `4.0`, hold `5.0` seconds
- `GOTO [0.0, -20.0, -20.0]`, speed `4.0`

Automated command:

```bash
cd /home/skink/projects/voloxide_setup/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/voloxide_setup/workspace/install/setup.zsh
source target/ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
python3 scripts/sim_roscopter_waypoint_acceptance.py \
  --use-rviz \
  --mission four \
  --observe-seconds 180 \
  --waypoint-tolerance 3.0
```

Latest observed result from the current branch:

```text
targets_ned=[(0.0, 0.0, -10.0), (20.0, 0.0, -10.0), (20.0, -20.0, -20.0), (0.0, -20.0, -20.0)]
waypoint_start_distances=[9.848, 22.288, 34.569, 28.201]
waypoint_min_distances=[4.82, 13.471, 20.486, 25.126] tolerance=3.000
max_command_thrust=44.900
status: armed=True failsafe=False offboard=True control_mode=0 rc_override=0 error_code=8
roscopter_counts: waypoints=4 trajectory=369 high_level=404 command=405
last_high_level: mode=10 valid=True cmd=(0.254,1.258,0.109,754.568)
max_pwm_delta=1000
last_command: mode=0 ignore=0 u0_3=[0.0, 0.0, -44.9, 0.0]
ROSCOPTER WAYPOINT RESPONSE FAILED
```

This confirms the current branch is not yet following the tutorial waypoints. The command path is
alive, but the vehicle fails the distance-based acceptance criteria.
