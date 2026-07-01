# Quad-X Angle-Mode Waypoint Example

This example runs ROScopter waypoint autonomy while sending ROSflight firmware
angle/throttle commands instead of ROScopter's normal mixer-passthrough command.
It is useful for validating autonomy with the canned quad-X firmware mixer.

The runner assumes the usual local development layout where `Veloxity` and
`rosflight/rosflight/workspace` share the same parent directory.

The important distinction is:

- ROSflight firmware angle mode is `/command.mode=2`
  (`MODE_ROLL_PITCH_YAWRATE_THROTTLE`).
- ROScopter mode `4` is a high-level ROScopter command enum, not the firmware
  angle mode.

## What This Runs

- Switches firmware params to canned quad-X for the experiment:
  `PRIMARY_MIXER=2`, `USE_MOTOR_PARAM=0`.
- Starts ROScopter `estimator`, `path_manager`, and `path_planner`.
- Does not start the stock ROScopter `controller` or `trajectory_follower`.
- Runs `trajectory_to_angle_command.py`, which consumes `/trajectory_command`
  and `/estimated_state`, preserves ROScopter path-planner position/velocity/
  acceleration feedforward, and publishes ROSflight `/command.mode=2`.
- Records a rosbag for analysis.
- Restores the recommended mixer on cleanup:
  `PRIMARY_MIXER=11`, `USE_MOTOR_PARAM=1`.

## Files

- `run_waypoint_angle_experiment.zsh`: launch, preflight checks, bag recording,
  cleanup, and mixer restoration.
- `clean_slate.zsh`: stop both the example-owned processes and the visual
  SIL/RViz stack before a fresh run.
- `cleanup_stale_processes.zsh`: stop stale example-owned ROScopter/GCS/bag
  processes after an interrupted run.
- `trajectory_to_angle_command.py`: final-stage planner-feedforward to
  angle/throttle controller.
- `angle_waypoint_baseline.yaml`: controller gains and safety limits.
- `analyze_waypoint_angle_bag.py`: bag metrics for command modes, PWM
  saturation, trajectory tracking, and estimator error.
- `publish_mission_markers.py`: publish the mission YAML as persistent RViz
  waypoint markers.
- `quadx_angle_hold.py`: simpler direct hover/hold test for firmware angle mode.

## Full Visual Run

### 1. Start From a Clean Slate

Run these commands from the directory that contains both `Veloxity` and
`rosflight`.

Stop any leftover processes from a previous example run and restart the visual
stack from scratch:

```bash
Veloxity/examples/quadx_angle_waypoints/clean_slate.zsh
```

Verify that no example or visual processes remain:

```bash
ps -eo pid,args | grep -E 'multirotor_standalone_sil|rviz2|standalone_viz_transcriber|rosflight_sil_manager|veloxity_sil_board|standalone_sensors|rosflight_io|rc.py|multirotor_forces_and_moments|standalone_dynamics|roscopter (estimator|path_manager|path_planner)|/roscopter/(estimator|path_manager|path_planner)|trajectory_to_angle_command|rviz_waypoint_publisher|ros2 bag record' | grep -v grep
```

No output from that command means the slate is clean.

Also verify that the ROSflight UDP port is free:

```bash
ss -lunp | grep 14520
```

No output from that command means nothing is still bound to the local
ROSflight I/O port. This check matters because an old `rosflight_io` process on
`127.0.0.1:14520` can let RViz open while the new simulator is actually broken.

### 2. Source Each New Terminal

Open each terminal from the directory that contains both `Veloxity` and
`rosflight`, then source ROS plus both local workspaces:

```bash
source /opt/ros/humble/setup.zsh
source rosflight/rosflight/workspace/install/setup.zsh
source Veloxity/workspace/install/setup.zsh
cd Veloxity
```

### 3. Terminal 1: Start Visual SIL/RViz

In the first terminal, start the visual SIL/RViz session:

```bash
ros2 launch veloxity_sil_board_shim multirotor_standalone_sil.launch.py use_rviz:=true
```

Wait until the terminal prints `veloxity_sil_board ready` and the RViz window is
visible. Also watch the first few seconds of output: `rosflight_io` must not
print `bind: Address already in use`. The RViz config already contains displays
for the vehicle mesh, vehicle path, and waypoint markers.

### 4. Terminal 2: Initialize Firmware/Baro

In a second terminal, initialize firmware/baro before each clean capture:

```bash
ros2 launch veloxity_sil_board_shim veloxity_multirotor_init_firmware.launch.py write_delay:=3.0
```

Confirm the firmware is healthy before flying:

```bash
ros2 topic echo /status --once
```

The status should show `armed: false`, `failsafe: false`, and `error_code: 0`.

### 5. Terminal 3: Run the Waypoint Example

In a third terminal, run the waypoint angle-mode example:

```bash
./examples/quadx_angle_waypoints/run_waypoint_angle_experiment.zsh \
  --auto-release \
  --duration 120 \
  --bag-name takeoff_logs/quadx_waypoint_angle_mode_full
```

The runner refuses to fly if stale experiment processes are present or if
`/estimated_state.p_d` is not near ground during preflight. That check matters:
a stale estimator/path stack can make `/trajectory_command` jump ahead in the
mission before takeoff.

RViz already has a marker display for `/rviz/waypoint`. The runner starts
`roscopter_gcs rviz_waypoint_publisher`, so the loaded waypoints should appear
as red spheres with text labels and a green line connecting the mission path.

To keep the waypoint markers visible before or after the flight runner exits,
open another sourced terminal and run:

```bash
./examples/quadx_angle_waypoints/publish_mission_markers.py \
  ../rosflight/rosflight/workspace/src/roscopter/roscopter/params/multirotor_mission.yaml
```

Leave that process running while you inspect RViz.

## Analyze

```bash
./examples/quadx_angle_waypoints/analyze_waypoint_angle_bag.py \
  takeoff_logs/quadx_waypoint_angle_mode_full
```

Known-good reference bag from July 1, 2026:

```bash
./examples/quadx_angle_waypoints/analyze_waypoint_angle_bag.py \
  takeoff_logs/quadx_waypoint_angle_mode_full_20260701
```

Reference result:

- All post-release commands used firmware angle mode: `{2: 12025}`.
- Lateral tracking error: mean `0.98 m`, p95 `2.58 m`, max `2.87 m`.
- Altitude error: mean `0.92 m`, p95 absolute `1.96 m`, max absolute `2.34 m`.
- Estimator altitude error: mean `-0.52 m`, p95 absolute `0.69 m`.
- PWM saturation samples: `7`.

## Operational Notes

- Keep the visual SIL/RViz session running while the experiment runs.
- Re-run firmware/baro initialization before each definitive capture.
- If a command fails with `ros2: command not found` or a package cannot be
  found, that terminal has not sourced the three setup files listed above.
- If `/status.error_code` remains nonzero after firmware init, especially
  `error_code: 40`, restart the visual SIL/RViz launch and then run firmware
  init again. A stale SIL stack can refuse to arm even though the arm service
  call returns success.
- Do not use the default ROScopter passthrough controller with canned quad-X;
  the passthrough force units do not match the canned normalized mixer input.
- If the runner exits with stale-process or preflight-estimator errors, clean up
  the old `estimator`, `path_manager`, `path_planner`, or bag process before
  rerunning.
- The runner verifies `armed: true` before loading the mission and verifies
  `rc_override: 0` before the timed flight segment.
- The `filtered` fields printed by `trajectory_to_angle_command.py` are only
  relevant when `use_filtered_target: true`. The default example uses planner
  feedforward directly.

## Troubleshooting Stale Processes

If the runner says stale experiment processes are present, inspect them:

```bash
ps -eo pid,args | grep -E 'roscopter (estimator|path_manager|path_planner)|/roscopter/(estimator|path_manager|path_planner)|trajectory_to_angle_command|rviz_waypoint_publisher|ros2 bag record' | grep -v grep
```

Then stop only those example-owned processes:

```bash
./examples/quadx_angle_waypoints/cleanup_stale_processes.zsh
```

This helper intentionally does not stop the visual SIL/RViz stack. It only
targets the estimator, path manager, path planner, waypoint marker publisher,
temporary angle controller, and rosbag process used by this example.

If `/status.error_code` stays nonzero after firmware init, the mesh does not
load, RViz disappears, or the vehicle will not arm, run the full clean-slate
helper from the parent directory and restart from Terminal 1:

```bash
Veloxity/examples/quadx_angle_waypoints/clean_slate.zsh
```

After restarting SIL/RViz, run firmware/baro init again before flying.
