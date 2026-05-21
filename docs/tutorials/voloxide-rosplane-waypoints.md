# Voloxide ROSplane Fixed-Wing Demo

This guide runs the Voloxide/Rust firmware backend through ROSflight 2.0
fixed-wing standalone SIL and the ROSplane waypoint stack. The ROSflight C
firmware, ROSflight sim, and ROSplane packages remain unchanged; Voloxide only
replaces the SIL board firmware endpoint.

It follows the upstream ROSflight ROSplane sim flow:

- launch fixed-wing standalone sim,
- launch ROSplane autonomy,
- load `fixedwing_mission.yaml`,
- arm and release RC override.

## Prerequisites

From the workspace root:

```bash
cd /run/host/home/skink/projects/voloxide_proj
source scripts/source_rosflight_env.zsh
source install/setup.zsh
```

Build the Voloxide sim library and ROS shim if needed:

```bash
cd Voloxide
cargo build -p sim --lib
cd ..
colcon build --base-paths Voloxide/sim/ros2/voloxide_sil_board_shim \
  --packages-select voloxide_sil_board_shim
source install/setup.zsh
```

Zenoh RMW is recommended for local graph stability:

```bash
export ROS_LOG_DIR=/tmp/rosflight_logs
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

## Working Visual Demo

Run:

```bash
cd /run/host/home/skink/projects/voloxide_proj
Voloxide/scripts/run_voloxide_rosplane_demo.zsh
```

This is the fixed-wing path that has been validated visually with Voloxide as
the firmware endpoint. It deliberately uses VimFly for the aircraft takeoff and
RC-override handoff, matching the ROSflight tutorial model more closely than the
diagnostic deterministic RC helper.

The script defaults to the validated tutorial mode:

- `FIRMWARE=voloxide`
- `USE_VIMFLY=true`
- `USE_TRUTH_STATE_AUTONOMY=true`
- `USE_STANDALONE_RVIZ=true`
- `USE_WAYPOINT_VIZ=true`
- `USE_ROSPLANE_GCS=false`
- `MANUAL_TAKEOFF_BEFORE_ROSPLANE=true`
- `RESET_VOLOXIDE_PARAMS=true`

It starts the Zenoh router, launches fixed-wing standalone SIL with VimFly,
seeds a ground state for firmware calibration, loads the standard ROSflight
fixed-wing firmware parameters, refreshes the dynamics parameter cache, starts
one RViz window, and then pauses.

At the first pause:

1. Click the VimFly window.
2. Press `t` once to arm.
3. Fly the aircraft manually under RC override.
4. Do not press `r` yet.
5. Press Enter in the script terminal only after the aircraft is airborne and
   stable.

The script then starts ROSplane from the truth-state adapter, starts the
waypoint marker publisher, loads `fixedwing_mission.yaml`, and pauses again.

At the second pause:

1. Keep the aircraft flying manually.
2. Press `r` once in VimFly to release RC override.
3. Press Enter in the script terminal after `/status` shows `rc_override=0`.

The standalone RViz window should show `/rviz/waypoint`, `/rviz/mesh`, and
`/rviz/mesh_path`. The separate ROSplane GCS RViz window is disabled by default
so there is only one visual window.

By default the script deletes `/tmp/voloxide_rosplane_sim.params` before launch
and then loads the documented ROSflight fixed-wing parameter file, so saved
Voloxide parameters from earlier tests do not define the demo flight
configuration.

Stop the demo with `Ctrl-C` in the script terminal.

## Manual Sequence

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

In a second terminal with the same environment, launch fixed-wing SIL through
the Voloxide firmware endpoint:

```bash
ros2 launch voloxide_sil_board_shim fixedwing_standalone_sil.launch.py \
  firmware:=voloxide \
  use_rviz:=true \
  use_vimfly:=true
```

Initialize fixed-wing firmware parameters and IMU calibration:

```bash
ros2 launch rosflight_sim fixedwing_init_firmware.launch.py
```

After firmware initialization, use VimFly to take off manually before starting
ROSplane:

1. Click the VimFly window.
2. Press `t` once to arm.
3. Fly the aircraft manually under RC override until it is airborne and stable.
4. Do not press `r` yet.

Start ROSplane from the truth-state adapter and start waypoint visualization:

```bash
ros2 launch voloxide_sil_board_shim rosplane_truth_state_autonomy.launch.py
python3 Voloxide/scripts/rosplane_waypoint_markers.py
```

Load the default fixed-wing mission:

```bash
cd workspace/src/rosplane/rosplane/missions
ros2 service call /load_mission_from_file rosflight_msgs/srv/ParamFile \
  "{filename: $(pwd)/fixedwing_mission.yaml}"
```

Release RC override from the VimFly window:

1. Click the VimFly window.
2. Keep the aircraft flying manually.
3. Press `r` once to release RC override.

When this is successful, `/status` should show `armed: true`,
`rc_override: 0`, and `offboard: true`.

## What To Watch

Cadence:

```bash
ros2 topic hz /command
ros2 topic hz /sim/pwm_output
```

State and commands:

```bash
ros2 topic echo /estimated_state
ros2 topic echo /sim/truth_state
ros2 topic echo /controller_internals
ros2 topic echo /status
ros2 topic echo /airspeed
ros2 topic echo /gnss
```

Expected healthy fixed-wing run:

- `/command` publishes from ROSplane to ROSflight.
- `/sim/pwm_output` publishes near the SIL tick rate.
- `/status` reports fixed-wing firmware after `fixedwing_init_firmware.launch.py`
  loads `FIXED_WING=1`.
- The ROSplane GCS shows the fixed-wing mission waypoints.

The service shortcuts `/toggle_arm` and `/toggle_override` are only available
when `rc.py` is using its simulated joystick path. With `use_vimfly:=true`,
`rc.py` hands control to VimFly and does not create those services.

## Notes

This is a Voloxide-side launch and script layer only. It deliberately mirrors
the upstream ROSflight fixed-wing standalone launch while switching only the
firmware endpoint between `sil_board` and `voloxide_sil_board`.

The tutorial wrapper follows a simple fixed-wing visual flow: launch Voloxide
SIL with VimFly and RViz, calibrate on the ground, let the user take off
manually under RC override, then start the ROSplane controller/path stack from
the sim truth-state adapter, load the mission, and release RC override. A
fixed-wing aircraft is not a quadrotor hover case; the scripted deterministic
RC helper is useful for diagnostics, but it does not replace a clean manual
takeoff in the visual ROSplane tutorial path.
The script uses `/tmp/voloxide_rosplane_sim.params` as the Voloxide SIL
parameter store so fixed-wing mixer and airframe settings cannot leak into the
quadrotor demo.

The one-command wrapper separates firmware calibration, visual startup, and
handoff:

- `INITIAL_AIRSPEED` and `INITIAL_DOWN_POSITION` seed the pre-ROSplane firmware
  calibration state and default to ground at zero speed.
- `USE_TRUTH_STATE_AUTONOMY` defaults to true in the visual tutorial wrapper.
  This launches the ROSplane controller, path follower, path manager, and path
  planner against `/sim/rosplane/state` instead of the stock ROSplane EKF.
- `ROSPLANE_START_AIRSPEED` and `ROSPLANE_START_DOWN_POSITION` seed the state
  ROSplane sees before mission load and arming when automatic startup seeding is
  enabled. In the default VimFly tutorial path, `MANUAL_TAKEOFF_BEFORE_ROSPLANE`
  is true, so ROSplane is not started until after manual takeoff and the current
  manually flown state is left continuous for handoff.
- `RC_HANDOFF_RELEASE_AIRSPEED` and `RC_HANDOFF_RELEASE_DOWN_POSITION` seed the
  finite fixed-wing state immediately before RC override release. The tutorial
  defaults preserve the previously validated `17 m/s`, `down=-70 m` release
  state. Stock ROSplane estimator mode does not reseed this state during
  handoff by default, because its startup state is already continuous with the
  release state.
- `RC_HANDOFF_MANUAL_WARMUP_SECONDS` defaults to `0.0` for the scripted
  fixed-wing handoff. The helper uses RC override to arm, confirms ROSplane is
  publishing commands, and releases override immediately instead of trying to
  manually fly the aircraft before autonomy takes over.
