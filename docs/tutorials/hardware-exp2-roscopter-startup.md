# Hardware Experiment 2: ROScopter Startup Walkthrough

This runbook begins after Veloxity firmware and `rosflight_io` are connected.
It covers the complete ROS-side startup for the upstream trajectory-follower
experiment, from the ROScopter estimator through mission loading, command
validation, and release of physical RC override.

The hardware does not provide the simulator's `/toggle_arm` or
`/toggle_override` services. Arm, disarm, take RC override, and release RC
override with the physical transmitter.

Do not start the stock `roscopter.launch.py`. It starts an unremapped
trajectory follower and would conflict with the two experiment adapters.

## CM4 quick-start configuration

The following setup is intended for the `rustflight-pi` Raspberry Pi CM4. It
keeps aircraft-specific files outside the repository under
`~/.config/veloxity/airframes/3dquad` while running the adapter scripts from a
checkout at `$HOME/Veloxity`.

Before flight, audit `ros/hardware-exp2.yaml`. The relevant sections must use
the exact node names and ROS 2 parameter nesting shown below:

```yaml
/**:
  ros__parameters:
    gravity: 9.81
    mass: 0.821  # Replace whenever ready-to-fly mass changes.

/trajectory_velocity_adapter:
  ros__parameters:
    north_kp: 1.5
    north_kd: 3.55
    east_kp: 1.5
    east_kd: 3.5
    down_kp: 4.0
    down_kd: 3.5

/thrust_to_throttle_adapter:
  ros__parameters:
    equilibrium_throttle: REPLACE_WITH_MEASURED_NUMBER
    min_throttle: REPLACE_WITH_VALIDATED_NUMBER
    max_throttle: REPLACE_WITH_VALIDATED_NUMBER
    max_roll_rad: 0.30
    max_pitch_rad: 0.30
    max_yaw_rate_rad_s: 0.70

/controller:
  ros__parameters:
    equilibrium_throttle: REPLACE_WITH_SAME_MEASURED_NUMBER
    min_throttle: REPLACE_WITH_SAME_VALIDATED_NUMBER
    max_throttle: REPLACE_WITH_SAME_VALIDATED_NUMBER
```

Replace every placeholder with a YAML number before starting any node. In
particular, do not use `trajectory_veloxity_adapter`, `ros__params`, or a
parameter section without the `ros__parameters` key. Do not copy the
simulation hover throttle onto the aircraft without hardware measurements.

The CM4 `setup.bash` should export at least these locations:

```bash
export VELOXITY_ROOT="$HOME/Veloxity"
export ROSFLIGHT_WS="$HOME/ROSflight_veloxity"

source /opt/ros/humble/setup.bash
source "$ROSFLIGHT_WS/install/local_setup.bash"

export ROSCOPTER_SHARE="$(ros2 pkg prefix roscopter)/share/roscopter"
export MULTIROTOR="$ROSCOPTER_SHARE/params/multirotor.yaml"
export ESTIMATOR="$ROSCOPTER_SHARE/params/estimator.yaml"

export AIRFRAME_CONFIG="$HOME/.config/veloxity/airframes/3dquad"
export FIRMWARE_PARAMS="$AIRFRAME_CONFIG/firmware/firmware-startup.yaml"
export EXPERIMENT="$AIRFRAME_CONFIG/ros/hardware-exp2.yaml"
export ESTIMATOR_HW="$AIRFRAME_CONFIG/ros/estimator-hardware.yaml"
export MISSION="$AIRFRAME_CONFIG/missions/hover-check.yaml"
export FLIGHT_LOG_ROOT="$HOME/flight-logs"

export ROSFLIGHT_UART=/dev/ttyAMA0
export ROSFLIGHT_USB=/dev/ttyACM0
export ROSFLIGHT_BAUD=921600
```

Add the following experiment functions to the airframe's `commands.bash` in
addition to `start_uart`, `start_usb`, firmware parameter, calibration,
mission, status, and bag helpers:

```bash
start_estimator() {
  ros2 run roscopter estimator --ros-args \
    -r __node:=estimator \
    --params-file "$ESTIMATOR" \
    --params-file "$ESTIMATOR_HW"
}

start_path_manager() {
  ros2 run roscopter path_manager --ros-args \
    -r __node:=path_manager \
    --params-file "$MULTIROTOR" \
    --params-file "$EXPERIMENT" \
    -r estimated_state:=estimated_state
}

start_path_planner() {
  ros2 run roscopter path_planner --ros-args \
    -r __node:=path_planner \
    --params-file "$MULTIROTOR" \
    -r estimated_state:=estimated_state
}

start_velocity_adapter() {
  python3 \
    "$VELOXITY_ROOT/examples/quadx_upstream_angle_waypoints/trajectory_velocity_adapter.py" \
    --ros-args \
    --params-file "$EXPERIMENT"
}

start_trajectory_follower() {
  ros2 run roscopter trajectory_follower --ros-args \
    -r __node:=trajectory_follower \
    --params-file "$MULTIROTOR" \
    --params-file "$EXPERIMENT" \
    -r estimated_state:=estimated_state \
    -r trajectory_command:=trajectory_command_compensated \
    -r high_level_command:=high_level_command_thrust
}

start_throttle_adapter() {
  python3 \
    "$VELOXITY_ROOT/examples/quadx_upstream_angle_waypoints/thrust_to_throttle_adapter.py" \
    --ros-args \
    --params-file "$MULTIROTOR" \
    --params-file "$EXPERIMENT"
}

start_controller() {
  ros2 run roscopter controller --ros-args \
    -r __node:=controller \
    --params-file "$MULTIROTOR" \
    --params-file "$EXPERIMENT" \
    -r estimated_state:=estimated_state
}

print_mission() {
  ros2 service call \
    /path_planner/print_waypoints \
    std_srvs/srv/Trigger \
    '{}'

  ros2 service call \
    /path_manager/print_waypoints \
    std_srvs/srv/Trigger \
    '{}'
}
```

After changing either shell file, reload it with `source ~/.bashrc`. Each
long-running function below occupies its terminal. Use one terminal or tmux
pane per line, with physical RC override active for the entire startup:

```text
Terminal 1: start_uart                 # Or start_usb, never both.
Terminal 2: start_estimator
Terminal 3: start_path_manager
Terminal 4: start_path_planner
Terminal 5: start_velocity_adapter
Terminal 6: start_trajectory_follower
Terminal 7: start_throttle_adapter
Terminal 8: start_controller
Terminal 9: monitoring, mission services, and start_bag
```

Once `rosflight_io` is connected, `load_firmware_params` may be used while
disarmed to load the reviewed aircraft configuration. Check the service
response and `/status`. Do not make `write_firmware_params` a routine startup
step: use it only when intentionally persisting the current in-memory firmware
parameters.

After all eight nodes are healthy, use Terminal 9 to run the graph checks in
Section 12, followed by:

```bash
load_mission
print_mission
start_bag
```

Require the planner and manager to print the same reviewed mission. Then use
the physical transmitter to arm while retaining override, wait for estimator
initialization, validate every command boundary, and apply the release gate in
Sections 15--17. Never combine the final arm and override-release actions into
an unattended shell function.

## 1. Understand the command path

```text
firmware sensors through rosflight_io
  /imu/data, /baro, /magnetometer, /gnss, /status
                 |
                 v
        ROScopter estimator
                 |
          /estimated_state
                 |
                 v
 path_planner -> /waypoints -> path_manager
                               |
                     /trajectory_command
                               |
                trajectory_velocity_adapter
                               |
               /trajectory_command_compensated
                               |
                     trajectory_follower
                               |
                /high_level_command_thrust
                               |
                 thrust_to_throttle_adapter
                               |
                   /high_level_command
                               |
                     ROScopter controller
                               |
                          /command
                               |
                           rosflight_io
                               |
                         Veloxity firmware
```

## 2. Prepare every terminal

Run this setup in every new terminal:

```bash
cd /home/skink/projects/ROSflight_ubuntu22/.distrobox-home/ROSflight_ubuntu22/Veloxity

source /opt/ros/humble/setup.zsh
source ../rosflight/rosflight/workspace/install/setup.zsh

export ROSCOPTER_SHARE="$(ros2 pkg prefix roscopter)/share/roscopter"
export MULTIROTOR="$ROSCOPTER_SHARE/params/multirotor.yaml"
export ESTIMATOR="$ROSCOPTER_SHARE/params/estimator.yaml"
export EXPERIMENT="$PWD/examples/quadx_upstream_angle_waypoints/upstream_angle_baseline.yaml"
```

The ROSflight workspace path is an example of the caller-provided workspace.
Use the workspace that was built and sourced for the hardware session.

For hardware, define the real vehicle values in the terminals that run the
trajectory follower, throttle adapter, and controller:

```bash
export MASS_KG=<measured-aircraft-mass>
export HOVER_THR=<measured-normalized-hover-throttle>
export MIN_THR=<approved-minimum-throttle>
export MAX_THR=<approved-maximum-throttle>
```

Do not use the simulator's `HOVER_THR=0.686` without measuring the aircraft.
The experiment YAML's mass, hover throttle, and throttle bounds are simulation
values, not validated hardware values.

## 3. Confirm the firmware interface

Before starting ROScopter:

```bash
ros2 topic echo /status --once
ros2 topic hz /imu/data
ros2 topic hz /baro
ros2 topic hz /magnetometer
ros2 topic hz /gnss
```

Use Ctrl-C to stop each rate measurement. The preflight status must show:

```text
armed: false
failsafe: false
error_code: 0
```

For an autonomous horizontal mission, `/gnss` should report at least a 3-D
fix:

```text
fix_type: 3
```

Do not continue if required sensor topics are absent, stale, or reporting
implausible values.

## 4. Terminal 1: start the ROScopter estimator

```bash
ros2 run roscopter estimator --ros-args \
  -r __node:=estimator \
  --params-file "$ESTIMATOR" \
  -p max_baro_sensor_silence_duration_ms:=60 \
  -p max_mag_sensor_silence_duration_ms:=60
```

The additional silence limits accommodate approximately 25 Hz Pixracer
barometer and magnetometer telemetry. The estimator consumes:

| Input | Purpose |
| --- | --- |
| `/imu/data` | Acceleration and angular velocity |
| `/baro` | Vertical-position correction |
| `/magnetometer` | Heading correction |
| `/gnss` | Position, velocity, and geographic origin |
| `/status` | Detection of the first arm event |

It publishes `/estimated_state` as `roscopter_msgs/msg/State`. Check it from a
monitoring terminal:

```bash
ros2 topic hz /estimated_state
ros2 topic echo /estimated_state --once
```

Expect approximately 390 Hz. Mostly zero state values before the first arm are
normal: the current estimator begins estimating after it first sees
`armed: true`.

## 5. Terminal 2: start the path manager

```bash
ros2 run roscopter path_manager --ros-args \
  -r __node:=path_manager \
  --params-file "$MULTIROTOR" \
  --params-file "$EXPERIMENT" \
  -r estimated_state:=estimated_state
```

The path manager consumes `/estimated_state` and `/waypoints`, then publishes
`/trajectory_command`. The experiment parameters set a 0.5 m waypoint
tolerance, 1.5 m/s maximum velocity, and 1.0 m/s^2 maximum acceleration.

Before a mission is loaded, the manager can publish its default trajectory.
Keep physical RC override active throughout setup.

## 6. Terminal 3: start the path planner

```bash
ros2 run roscopter path_planner --ros-args \
  -r __node:=path_planner \
  --params-file "$MULTIROTOR" \
  -r estimated_state:=estimated_state
```

The planner loads mission YAML files, converts LLA waypoints to NED when
requested, and publishes `/waypoints`. Confirm its services exist:

```bash
ros2 service list | rg 'path_planner|path_manager'
```

## 7. Terminal 4: start the velocity feed-forward adapter

```bash
python3 \
  examples/quadx_upstream_angle_waypoints/trajectory_velocity_adapter.py \
  --ros-args \
  --params-file "$EXPERIMENT"
```

This node consumes `/trajectory_command` and publishes
`/trajectory_command_compensated`. The upstream trajectory follower does not
consume the velocity reference. The adapter advances only its private position
reference by `kd / kp * commanded_velocity`, restoring velocity feed-forward
while leaving `/trajectory_command` unchanged for plotting and analysis.

## 8. Terminal 5: start the trajectory follower

Set `MASS_KG` in this terminal, then run:

```bash
ros2 run roscopter trajectory_follower --ros-args \
  -r __node:=trajectory_follower \
  --params-file "$MULTIROTOR" \
  --params-file "$EXPERIMENT" \
  -p mass:="$MASS_KG" \
  -r estimated_state:=estimated_state \
  -r trajectory_command:=trajectory_command_compensated \
  -r high_level_command:=high_level_command_thrust
```

It consumes `/estimated_state`, `/trajectory_command_compensated`, and
`/status`. It publishes `/high_level_command_thrust`. A healthy message has:

```text
mode: 10
cmd_valid: true
```

For mode 10, `cmd1` is roll in radians, `cmd2` is pitch in radians, `cmd3` is
yaw rate in radians per second, and `cmd4` is thrust in newtons.

```bash
ros2 topic hz /high_level_command_thrust
ros2 topic echo /high_level_command_thrust --once
```

All values must be finite. Near steady hover, thrust should be physically
plausible and near `MASS_KG * 9.81` newtons.

## 9. Terminal 6: start the thrust-to-throttle adapter

Set all four hardware values in this terminal, then run:

```bash
python3 \
  examples/quadx_upstream_angle_waypoints/thrust_to_throttle_adapter.py \
  --ros-args \
  --params-file "$MULTIROTOR" \
  --params-file "$EXPERIMENT" \
  -p mass:="$MASS_KG" \
  -p equilibrium_throttle:="$HOVER_THR" \
  -p min_throttle:="$MIN_THR" \
  -p max_throttle:="$MAX_THR"
```

It converts thrust to normalized throttle using:

```text
throttle = thrust_N / (mass * gravity) * hover_throttle
```

It consumes `/high_level_command_thrust` and publishes
`/high_level_command`. A healthy adapted message has:

```text
mode: 6
cmd_valid: true
```

The adapter limits roll and pitch to +/-0.30 rad, yaw rate to +/-0.70 rad/s,
and throttle to `MIN_THR..MAX_THR`.

```bash
ros2 topic hz /high_level_command
ros2 topic echo /high_level_command --once
```

Do not proceed if throttle remains at either limit. That indicates an invalid
mass/hover-throttle value, bad altitude estimate, or controller saturation.

## 10. Terminal 7: start the ROScopter controller

Set all four hardware values in this terminal, then run:

```bash
ros2 run roscopter controller --ros-args \
  -r __node:=controller \
  --params-file "$MULTIROTOR" \
  -p mass:="$MASS_KG" \
  -p equilibrium_throttle:="$HOVER_THR" \
  -p min_throttle:="$MIN_THR" \
  -p max_throttle:="$MAX_THR" \
  -r estimated_state:=estimated_state
```

The controller consumes `/estimated_state`, `/high_level_command`, and
`/status`, then publishes `/command` as `rosflight_msgs/msg/Command`.

The final expected firmware command uses:

```text
mode: 2
ignore: 0
```

For mode 2, `u[2]` is normalized throttle, `u[3]` is roll in radians, `u[4]`
is pitch in radians, and `u[5]` is yaw rate in radians per second. The other
entries normally remain zero.

## 11. Terminal 8: start RViz and hardware visualization

```bash
ros2 launch roscopter_gcs roscopter_gcs.launch.py
```

This starts RViz, the frame transforms, waypoint visualization, and aircraft
visualization.

## 12. Verify the complete ROS graph

```bash
ros2 node list
```

The graph should contain at least:

```text
/rosflight_io
/estimator
/path_manager
/path_planner
/trajectory_velocity_adapter
/trajectory_follower
/thrust_to_throttle_adapter
/controller
```

Inspect every boundary:

```bash
ros2 topic info /estimated_state --verbose
ros2 topic info /trajectory_command --verbose
ros2 topic info /trajectory_command_compensated --verbose
ros2 topic info /high_level_command_thrust --verbose
ros2 topic info /high_level_command --verbose
ros2 topic info /command --verbose
```

Require exactly one publisher along the custom command chain. In particular:

- `/trajectory_command_compensated`: velocity adapter to follower
- `/high_level_command_thrust`: follower to throttle adapter
- `/high_level_command`: throttle adapter to controller
- `/command`: controller to `rosflight_io`

If `/high_level_command` has two publishers, an unremapped follower or stock
ROScopter launch is also running. Stop and clean up before continuing.

## 13. Start the all-topic bag before arming

```bash
mkdir -p takeoff_logs

ros2 bag record -a \
  -o "takeoff_logs/hardware_exp2_$(date +%Y%m%d_%H%M%S)"
```

Leave the recorder running through arming, ROScopter barometer initialization,
mission release, and flight. Stop it cleanly with Ctrl-C after the vehicle is
safe and disarmed.

## 14. Load and verify the mission while override is active

Use an absolute path to a hardware-approved mission:

```bash
export MISSION=/absolute/path/to/hardware_mission.yaml
```

Do not use the simulation mission without review: it contains 20 m legs and a
40 m final altitude. Load the approved mission:

```bash
ros2 service call \
  /path_planner/load_mission_from_file \
  rosflight_msgs/srv/ParamFile \
  "{filename: '$MISSION'}"
```

Require `success: true`. Then inspect both the planner's stored list and the
manager's active list:

```bash
ros2 service call \
  /path_planner/print_waypoints \
  std_srvs/srv/Trigger "{}"

ros2 service call \
  /path_manager/print_waypoints \
  std_srvs/srv/Trigger "{}"
```

Both lists must match. NED waypoint coordinates are `[north, east, down]`;
negative down is above the initialized origin.

## 15. Arm with physical RC override active

Before arming, require:

- physical override switch active;
- throttle low and sticks centered;
- `/status.failsafe == false`;
- `/status.error_code == 0`;
- mission verified in both planner and manager;
- bag recording active.

Arm from the transmitter, then check:

```bash
ros2 topic echo /status --once
```

Require:

```text
armed: true
failsafe: false
rc_override: <nonzero>
error_code: 0
```

A nonzero override value is expected while the RC switch intentionally retains
manual control.

## 16. Wait for estimator initialization and validate `/command`

ROScopter begins its own barometer calibration after the first arm. This is
separate from firmware barometer calibration. At approximately 25 Hz and 100
samples, one hardware attempt takes about four seconds. Keep the vehicle still
and retain override for at least 6--10 seconds.

If the estimator logs:

```text
Bad baro calibration. Recalibrating
```

do not release override. Wait through another complete calibration window.
Monitor `/estimated_state` and require stable, plausible state:

```bash
ros2 topic echo /estimated_state
```

On the ground, `p_d` should be close to zero and no longer drifting, `v_z`
should be near zero, roll and pitch should match the stationary vehicle, and
horizontal position/velocity should be sensible. With GNSS, the initial
latitude, longitude, and altitude should be populated.

Use Ctrl-C, then validate all command levels:

```bash
ros2 topic echo /high_level_command_thrust --once
ros2 topic echo /high_level_command --once
ros2 topic echo /command --once
ros2 topic hz /command
```

Require:

- `/high_level_command_thrust`: mode 10 and `cmd_valid: true`;
- `/high_level_command`: mode 6 and `cmd_valid: true`;
- `/command`: mode 2, ignore 0, finite and stable values;
- `/command.u[2]` inside `MIN_THR..MAX_THR`, not pinned at a limit;
- `/command.u[3]` and `u[4]` inside +/-0.30 rad;
- `/command.u[5]` inside +/-0.70 rad/s;
- continuous command publication near the estimator update rate.

`rosflight_msgs/msg/Command` has no `cmd_valid` field. Validate the two
upstream `ControllerCommand` messages first, then validate `/command` mode,
ignore mask, freshness, and numerical bounds.

Once armed with valid input, the ROScopter controller prepares an internal
takeoff to `p_d=-2 m`, followed by a short position hold, before entering the
mission trajectory. This takeoff is the expected first response after release.

## 17. Apply the final go/no-go gate and release override

Do not release unless every condition is true:

- `/status.armed == true`;
- `/status.failsafe == false`;
- `/status.error_code == 0`;
- `/status.offboard == true`;
- override is active only because the switch is intentionally held;
- GNSS has the required fix;
- `/estimated_state` is stable;
- no rejected barometer calibration is in progress;
- the mission printed correctly from planner and manager;
- mode 10 and mode 6 commands are valid;
- `/command` is mode 2, ignore 0, finite, bounded, and updating continuously;
- bag recording is active.

Release the physical RC override switch while keeping sticks centered. Check
immediately:

```bash
ros2 topic echo /status --once
```

Require:

```text
armed: true
failsafe: false
rc_override: 0
offboard: true
error_code: 0
```

`rc_override: 0` confirms that Veloxity is accepting the computer command. If
it remains nonzero, restore intentional override and diagnose the bitfield:

| Bit | Meaning |
| ---: | --- |
| 1 | Attitude-override switch |
| 2 | Throttle-override switch |
| 4 | RC X-stick deviation |
| 8 | RC Y-stick deviation |
| 16 | RC Z-stick deviation |
| 32 | RC throttle deviation |
| 64--512 | One or more offboard command axes are inactive |

Do not troubleshoot an unexpected override state while continuing autonomous
flight. Retake RC override, stabilize or land, and inspect the recorded topics.

## Related files

- [Experiment README](../../examples/quadx_upstream_angle_waypoints/README.md)
- [Simulation orchestration](../../examples/quadx_upstream_angle_waypoints/run_upstream_angle_experiment.zsh)
- [Experiment journal](../../examples/quadx_upstream_angle_waypoints/EXPERIMENT_LOG.md)
- [Pixracer Pro guide](../boards/stm32.md)
