# Voloxide ROScopter Waypoint Demo

This guide starts from the checked-out `voloxide_proj` workspace and runs the Voloxide/Rust firmware
backend through the ROSflight 2.0 SIL and ROScopter waypoint stack.

It builds on [Connect Voloxide Firmware To ROSflight SIL](voloxide-firmware-bridge.md). Complete the
bridge tutorial first if you have not already verified that `firmware:=voloxide` is connected and
publishing `/status` and `/sim/pwm_output`.

The validated setup uses:

- Voloxide firmware backend through `voloxide_sil_board`.
- ROSflight standalone multirotor dynamics, sensors, RC, and RViz.
- ROScopter estimator, controller, trajectory follower, path manager, path planner, and RViz waypoint
  markers.
- Zenoh RMW for more stable local ROS 2 graph timing.
- Stock standalone sensor simulation, including barometer random bias and random walk.
- Estimator `rho` left as `NOT_IN_USE` so ROScopter computes air density from GNSS altitude.

## Prerequisites

From the workspace root:

```bash
cd /run/host/home/skink/projects/voloxide_proj
source scripts/source_rosflight_env.zsh
source install/setup.zsh
```

Install Zenoh RMW if it is not already installed:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-rmw-zenoh-cpp
```

Build the Voloxide sim library and ROS shim, if you have not already done so:

```bash
cd Voloxide
cargo build -p sim --lib
cd ..
colcon build --base-paths Voloxide/ros2/voloxide_sil_board_shim \
  --packages-select voloxide_sil_board_shim
source install/setup.zsh
```

## One-Command Demo

Run:

```bash
cd /run/host/home/skink/projects/voloxide_proj
Voloxide/scripts/run_voloxide_waypoint_demo.zsh
```

The script starts the Zenoh router, launches the Voloxide firmware bridge with RViz, runs firmware
initialization and IMU calibration, starts ROScopter's estimator first, arms and waits for stationary
barometer calibration, starts the ROScopter autonomy nodes and waypoint marker publisher, sets
`/path_manager hold_last=true`, loads the default multirotor mission, and releases RC override.
It uses `/tmp/voloxide_roscopter_sim.params` as the Voloxide SIL parameter store so fixed-wing
simulation runs cannot leave stale mixer or airframe settings in the quadrotor demo.
By default it deletes that store before launch (`RESET_VOLOXIDE_PARAMS=true`) and then loads
`rosflight_sim/params/multirotor_firmware/multirotor_combined.yaml`, so the flight demo uses the
documented ROSflight parameter file rather than saved parameters from previous tests.
For this external-estimator sim path it also sets firmware parameter `FILT_USE_ACC=0` before arming,
so the firmware does not block actuator output on its internal accelerometer correction health gate
while ROScopter is already feeding external attitude.

Stop the demo with `Ctrl-C` in the script terminal.

## Manual Sequence

Use the manual sequence when debugging one stage at a time.

Set the environment:

```bash
cd /run/host/home/skink/projects/voloxide_proj
source scripts/source_rosflight_env.zsh
source install/setup.zsh
export ROS_LOG_DIR=/tmp/rosflight_logs
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Start the Zenoh router:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

In a second terminal with the same environment, launch the Voloxide firmware bridge with GUI:

```bash
ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py \
  firmware:=voloxide \
  use_rviz:=true
```

Run firmware initialization and calibration:

```bash
ros2 launch rosflight_sim multirotor_init_firmware.launch.py
ros2 service call /calibrate_imu std_srvs/srv/Trigger
ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: FILT_USE_ACC, value: 0.0}"
```

Start only the ROScopter estimator first. Override `rho` to `NOT_IN_USE`; the installed estimator
YAML currently sets `rho: 1.225`, which forces sea-level density and bypasses the documented
GNSS-altitude density calculation.

```bash
ros2 run roscopter estimator \
  --ros-args \
  --params-file workspace/install/roscopter/share/roscopter/params/estimator.yaml \
  -r imu:=/imu/data \
  -p hotstart_estimator:=false \
  -p rho:=-1000000.0
```

The `imu` remap is required when launching the estimator directly. ROSflight
publishes the bridged IMU on `/imu/data`; without the remap the estimator waits
on `/imu`, `/estimated_state` stays frozen at the origin, and the controller
publishes a zero `/command` even though the trajectory follower is active.

Arm and wait on the ground for ROScopter's barometer calibration:

```bash
ros2 service call /toggle_arm std_srvs/srv/Trigger
sleep 4
```

Start the ROScopter autonomy nodes:

```bash
ros2 run roscopter controller \
  --ros-args --params-file workspace/install/roscopter/share/roscopter/params/multirotor.yaml
ros2 run roscopter trajectory_follower \
  --ros-args --params-file workspace/install/roscopter/share/roscopter/params/multirotor.yaml
ros2 run roscopter path_manager \
  --ros-args --params-file workspace/install/roscopter/share/roscopter/params/multirotor.yaml
ros2 run roscopter path_planner \
  --ros-args --params-file workspace/install/roscopter/share/roscopter/params/multirotor.yaml
ros2 run roscopter_gcs rviz_waypoint_publisher
```

Configure the path manager and load the default mission:

```bash
ros2 param set /path_manager hold_last true
cd workspace/src/roscopter/roscopter/params
ros2 service call /path_planner/load_mission_from_file rosflight_msgs/srv/ParamFile \
  "{filename: $(pwd)/multirotor_mission.yaml}"
```

Release RC override to enable offboard control:

```bash
ros2 service call /toggle_override std_srvs/srv/Trigger
```

## What To Watch

Cadence:

```bash
ros2 topic hz /command
ros2 topic hz /sim/pwm_output
```

State and commands:

```bash
ros2 topic echo /trajectory_command
ros2 topic echo /estimated_state
ros2 topic echo /sim/truth_state
ros2 topic echo /status
ros2 topic echo /baro
ros2 topic echo /gnss
```

Expected healthy GUI run:

- `/command`: about 390 Hz.
- `/sim/pwm_output`: about 400 Hz.
- `/status.error_code`: usually `0` after transient startup/calibration errors clear.
- Final mission hold should command `trajectory_command.position[2] = -40`.
- With `rho` left as `NOT_IN_USE`, final estimate/truth vertical mismatch should be small compared
  with the previous sea-level-density run.

## Why This Sequence Matters

The ROSflight tutorial says RC override starts enabled and must be disabled for autonomous flight.
It also says `standalone_sim` can drift when armed because the ground-plane model is simple.
When `use_vimfly:=true`, the `rc.py` helper does not create the `/toggle_arm`
and `/toggle_override` service shortcuts; use VimFly's `t` and `r` keys instead.

ROScopter's estimator starts barometer calibration after the first armed status. Starting the
estimator alone, arming, and waiting before starting controller/path nodes prevents the controller
from commanding takeoff while barometer calibration is still being collected.

ROScopter's documentation says `rho` is normally calculated from GNSS altitude when left as
`NOT_IN_USE`. The local estimator parameter file currently sets `rho: 1.225`, which is sea-level
density. The sim origin is around 1387 m MSL, so forcing sea-level density causes a large pressure to
height scale error.

## Current Simplifications

The helper script handles the validated ordering and avoids the earlier manual terminal juggling.
The remaining candidate simplification is a launch file that starts ROScopter's estimator first and
then conditionally starts the rest of the autonomy stack after arming/calibration. That should be
done as a Voloxide-side launch or script only, not by editing ROSflight firmware.
