# Connect Voloxide Firmware To ROSflight SIL

This tutorial connects the Voloxide/Rust firmware backend to the ROSflight 2.0 software-in-the-loop
ecosystem while leaving the rest of the ROSflight and ROScopter architecture unchanged.

The goal is equivalence at the firmware boundary:

- If the upstream ROSflight C `sil_board` is selected, ROSflight sees a firmware endpoint.
- If Voloxide's `voloxide_sil_board` is selected, ROSflight should still see a firmware endpoint with
  the same ROS graph contracts.
- Everything above the firmware endpoint, including `rosflight_io`, standalone sensors/dynamics,
  RC, ROScopter estimator/controller/path nodes, and RViz, remains normal ROSflight 2.0 usage.

Do not edit ROSflight firmware or Voloxide firmware for this tutorial.

## What The Bridge Replaces

In the upstream standalone SIL stack, ROSflight uses:

```text
rosflight_sim/sil_board
```

Voloxide replaces that single SIL board process with:

```text
voloxide_sil_board_shim/voloxide_sil_board
```

Both are driven by `rosflight_sil_manager` through the `sil_board/run` service and interact with
`rosflight_io` through the same MAVLink UDP ports.

## Build

From the workspace root:

```bash
cd /run/host/home/skink/projects/voloxide_proj
source scripts/source_rosflight_env.zsh
source install/setup.zsh
```

Build the Voloxide host-side sim library:

```bash
cd Voloxide
cargo build -p sim --lib
cd ..
```

Build and install the ROS shim:

```bash
colcon build --base-paths Voloxide/ros2/voloxide_sil_board_shim \
  --packages-select voloxide_sil_board_shim
source install/setup.zsh
```

## Middleware

For this machine, Zenoh RMW has been more reliable than Fast DDS under GUI load:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOG_DIR=/tmp/rosflight_logs
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Run the router in its own terminal. Every ROS command in the rest of this tutorial should use:

```bash
source scripts/source_rosflight_env.zsh
source install/setup.zsh
export ROS_LOG_DIR=/tmp/rosflight_logs
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

## Launch The Voloxide Firmware Endpoint

Use the Voloxide bridge launch:

```bash
ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py \
  firmware:=voloxide \
  use_rviz:=true
```

This starts:

- `rosflight_sil_manager`
- `voloxide_sil_board`
- `standalone_sensors`
- `rosflight_io`
- `rc.py`
- multirotor forces/dynamics
- RViz visualization helpers when `use_rviz:=true`

To compare against upstream C firmware with the same launch shape:

```bash
ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py \
  firmware:=c \
  use_rviz:=true
```

The point of this launch file is to make `firmware:=voloxide` and `firmware:=c` differ only at the
SIL board endpoint.

## Initialize Firmware Parameters And IMU Calibration

Run the standard ROSflight firmware init sequence:

```bash
ros2 launch rosflight_sim multirotor_init_firmware.launch.py
```

You can explicitly rerun IMU calibration:

```bash
ros2 service call /calibrate_imu std_srvs/srv/Trigger
```

Watch the `rosflight_io` terminal for:

```text
Gyro Calibration complete!
Accelerometer Calibration Complete!
Autopilot RECOVERED ERROR: Uncalibrated IMU
```

## Smoke Tests

Confirm ROSflight sees the firmware endpoint:

```bash
ros2 topic echo --once /status rosflight_msgs/msg/Status
ros2 topic hz /sim/pwm_output
```

Expected:

- `/status` publishes.
- `/sim/pwm_output` publishes near 400 Hz while the SIL manager is ticking.
- Startup transients such as RC lost or uncalibrated IMU clear after RC and calibration settle.

At this point Voloxide is connected as the firmware backend. Continue with normal ROSflight 2.0
manual workflows, or continue to the waypoint-following tutorial.

## Manual Control Boundary

The bridge does not itself run ROScopter waypoint following. It only provides the firmware endpoint.

From here you can use normal ROSflight 2.0 operations:

- `/toggle_arm`
- `/toggle_override`
- `/command`
- ROScopter estimator/controller/path nodes
- RViz and waypoint marker tools

This is the intended architectural separation: Voloxide should be interchangeable with the C
firmware endpoint below `rosflight_io`; autonomy and estimation remain ROSflight/ROScopter concerns.

