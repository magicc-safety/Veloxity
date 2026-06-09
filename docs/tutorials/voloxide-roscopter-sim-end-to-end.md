# Voloxide ROScopter Sim End-to-End

> [!summary]
> This guide runs the Voloxide Rust firmware inside the ROSflight standalone multirotor simulator,
> initializes that firmware through `rosflight_io`, then starts ROScopter autonomy and loads a
> waypoint mission.

> [!important]
> The words below are intentionally specific:
>
> - **Build the shim** means compile the Rust simulator library and the C++ ROS 2 bridge.
> - **Start the simulator** means launch ROSflight standalone sim, `rosflight_io`, and the Voloxide
>   Rust firmware endpoint.
> - **Initialize the running firmware** means load firmware params, calibrate IMU/baro, and write
>   params through `rosflight_io` services.
> - **Launch ROScopter** means start the autonomy stack after the firmware endpoint is already alive.

> [!note]
> These commands assume ROS 2 and the ROSflight workspace have already been sourced by your shell.
> Voloxide scripts use the caller's environment; they do not source external ROSflight helper
> scripts.

## Terminal Layout

Use separate terminals so long-running nodes stay visible.

| Terminal | Purpose |
|---|---|
| Terminal 1 | Build the shim, then keep the Voloxide simulator running |
| Terminal 2 | Initialize the running firmware |
| Terminal 3 | Launch ROScopter autonomy |
| Terminal 4 | Load missions, publish waypoints, arm, monitor |

## Phase 1: Build The Shim

**Terminal 1**

```bash
cd /home/skink/projects/ROSflight/.distrobox-home/ROSflight/Voloxide
source scripts/build_and_source_ros2_shim.zsh
```

This builds:

- `target/debug/libsim.a`: Rust sim firmware static library
- `voloxide_sil_board_shim`: ROS 2 C++ FFI bridge

Optional compile-only check:

```bash
cargo test -p sim
zsh scripts/build_and_source_ros2_shim.zsh
```

> [!tip]
> The first command should usually be run with `source` so the built overlay is available in the
> current terminal.

## Phase 2: Start The Simulator And Rust Firmware

**Terminal 1**

```bash
export ROS_LOG_DIR=/tmp/voloxide-ros-log

ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py \
  use_rviz:=true
```

The launch defaults to:

```bash
firmware:=voloxide
```

That means the simulator uses the Rust firmware endpoint:

```text
/voloxide_sil_board
```

instead of upstream C firmware:

```text
/sil_board
```

Both provide the `sil_board/run` service expected by `rosflight_sil_manager`.

Expected startup lines:

```text
voloxide_sil_board ready: service=sil_board/run, pwm=sim/pwm_output
rosflight_io: Connecting over UDP to "localhost:14525", from "localhost:14520"
rosflight_io: Got HEARTBEAT, connected.
rosflight_io: Received all parameters
```

> [!warning]
> Do not close Terminal 1 after this. It is the simulator and firmware process.

### Compare Against C Firmware

Use this only when you specifically want the upstream ROSflight C firmware endpoint:

```bash
ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py \
  firmware:=c \
  use_rviz:=false \
  use_builtin_rc:=false
```

## Phase 3: Initialize The Running Firmware

The firmware process is already running in Terminal 1. This phase sends setup commands to it through
`rosflight_io`.

**Terminal 2**

```bash
cd /home/skink/projects/ROSflight/.distrobox-home/ROSflight/Voloxide
source workspace/install/setup.zsh
```

Verify the simulator and Rust firmware endpoint are visible:

```bash
ros2 node list
```

You should see at least:

```text
/rosflight_io
/voloxide_sil_board
```

### Recommended: Voloxide Convenience Init

Run:

```bash
ros2 launch voloxide_sil_board_shim voloxide_multirotor_init_firmware.launch.py
```

This sends the following service calls in order:

1. `/param_load_from_file`
2. `/calibrate_imu`
3. `/calibrate_baro`
4. `/param_write`

> [!note]
> This is the Voloxide version of the ROSflight tutorial's convenience script. It adds barometer
> calibration, which the upstream ROSflight convenience launch does not perform.

Optional arguments:

```bash
ros2 launch voloxide_sil_board_shim voloxide_multirotor_init_firmware.launch.py \
  param_file:=/path/to/multirotor_combined.yaml \
  write_delay_s:=10
```

For fixed-wing:

```bash
ros2 launch voloxide_sil_board_shim voloxide_fixedwing_init_firmware.launch.py
```

### Manual Init Equivalent

Use this when debugging individual service calls.

```bash
cd /home/skink/projects/ROSflight/.distrobox-home/ROSflight/rosflight/workspace/src/rosflight_ros_pkgs/rosflight_sim/params

ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile \
  "{filename: $(pwd)/multirotor_firmware/multirotor_combined.yaml}"

ros2 service call /calibrate_imu std_srvs/srv/Trigger
ros2 service call /calibrate_baro std_srvs/srv/Trigger
ros2 service call /param_write std_srvs/srv/Trigger
```

For fixed-wing, load:

```bash
ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile \
  "{filename: $(pwd)/fixedwing_firmware.yaml}"
```

Watch Terminal 1 during init. You should see parameter traffic and the startup calibration errors
recover.

### Persistent Voloxide Params

Voloxide FFI sim parameters are saved through `VOLOXIDE_SIM_PARAM_DIR`.

The multirotor standalone launch defaults to:

```text
/tmp/voloxide-sim-params/multirotor
```

Use a persistent path if you want params to survive cleanup of `/tmp`:

```bash
ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py \
  voloxide_param_dir:=/some/persistent/path
```

## Phase 4: Launch ROScopter Autonomy

Keep Terminal 1 running.

> [!important]
> For service-based arming and override control, the standalone sim should be running with the built-in
> RC node enabled. That is the default. If you set it explicitly, use:
>
> ```bash
> use_builtin_rc:=true
> ```

**Terminal 3**

```bash
cd /home/skink/projects/ROSflight/.distrobox-home/ROSflight/Voloxide
source workspace/install/setup.zsh

ros2 launch roscopter_sim sim.launch.py
```

Expected ROScopter nodes include:

```text
/autopilot
/estimator
/external_attitude_transcriber
/path_manager
/path_planner
/roscopter_truth
/trajectory_follower
```

Check with:

```bash
ros2 node list
```

## Phase 5: Load A Mission

**Terminal 4**

```bash
cd /home/skink/projects/ROSflight/.distrobox-home/ROSflight/Voloxide
source workspace/install/setup.zsh
```

Optional waypoint visualization:

```bash
ros2 run roscopter_gcs rviz_waypoint_publisher
```

Load the default ROScopter mission:

```bash
cd /home/skink/projects/ROSflight/.distrobox-home/ROSflight/rosflight/workspace/src/roscopter/roscopter/params

ros2 service call /path_planner/load_mission_from_file rosflight_msgs/srv/ParamFile \
  "{filename: $(pwd)/multirotor_mission.yaml}"
```

Verify waypoints:

```bash
ros2 topic echo /waypoints
```

Publish the next queued waypoint:

```bash
ros2 service call /path_planner/publish_next_waypoint std_srvs/srv/Trigger
```

Publish all initial waypoints:

```bash
ros2 param set /path_planner num_waypoints_to_publish_at_start 100
```

Manual waypoint example:

```bash
ros2 service call /path_planner/add_waypoint roscopter_msgs/srv/AddWaypoint \
  "{wp: {type: 1, w: [5.0, 5.0, -4.0], speed: 4.0, psi: 0.0, use_lla: false}, publish_now: true}"
```

## Phase 6: Enable Autonomous Flight

If `rc.py` is running and you are not using VimFly or a transmitter:

```bash
ros2 service call /toggle_arm std_srvs/srv/Trigger
ros2 service call /toggle_override std_srvs/srv/Trigger
```

> [!warning]
> ROSflight starts with RC override enabled by default. Autonomy cannot control the vehicle until
> override is disabled.

## Phase 7: Monitor Flight

Useful topic echoes:

```bash
ros2 topic echo /estimated_state
ros2 topic echo /high_level_command
ros2 topic echo /command
ros2 topic echo /status
```

Useful rate checks:

```bash
ros2 topic hz /command
ros2 topic hz /sim/pwm_output
ros2 topic hz /imu/data
ros2 topic hz /sim/truth_state
```

If the vehicle drifts or you need to reset the standalone sim state:

```bash
ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState
```

## Debug Checks

Use these before blaming ROScopter:

```bash
ros2 topic echo /sim/truth_state --once
ros2 topic echo /imu/data --once
ros2 topic echo /baro --once
ros2 topic echo /status --once
ros2 topic echo /rc_raw --once
ros2 topic echo /sim/pwm_output --once
```

> [!bug]
> If `/sim/truth_state` is quiet but `/imu/data`, `/baro`, `/status`, or timestamps look impossible,
> the problem is likely in the firmware bridge or firmware telemetry path, not in ROScopter mission
> logic. `/imu/data` and other firmware telemetry should have normal ROS stamps after timesync; they
> should not show negative seconds.

## References

- ROSflight manual sim tutorial:
  <https://docs.rosflight.org/latest/user-guide/tutorials/manually-flying-rosflight-sim/>
- ROSflight ROScopter sim tutorial:
  <https://docs.rosflight.org/latest/user-guide/tutorials/setting-up-roscopter-in-sim/>
