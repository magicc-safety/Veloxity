# SIL Recovery Progress

## Goal

Recover a trustworthy ROSflight/ROScopter SIL workflow for Voloxide. The first milestone is to prove
the upstream ROSflight C firmware baseline before comparing Voloxide through the FFI SIL board.

The target tutorial flow is the ROScopter multirotor standalone simulation with RViz visualization,
firmware initialization, arming, override disable, mission loading, and rosbag capture for trajectory
analysis.

## Environment Findings

- The active repo is `/run/host/home/skink/projects/voloxide/Voloxide`.
- `main` was pulled from `origin/main` and was already up to date.
- The installed ROSflight workspace on this machine is:
  `/home/skink/projects/rosflight_setup/workspace`.
- The older documented path `/home/skink/projects/voloxide_setup/workspace` does not exist here.
- ROSflight source check: `rosflight_ros_pkgs`, `roscopter`, and `rosplane` are clean. No local
  ROSflight C-code changes were found to discard.
- ROS launch logging must use a writable directory:
  `ROS_LOG_DIR=/run/host/home/skink/projects/voloxide/Voloxide/target/ros2/roslog`.
- ROS2 and MAVLink UDP sockets do not work inside the sandbox. Fast DDS reports
  `getifaddrs: Operation not permitted` and Boost ASIO socket `open: Operation not permitted`.
- Running outside the sandbox with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` fixes the socket layer.
- `rmw_zenoh_cpp` is not installed on this machine, so Zenoh should not be used for the current
  C baseline runs.

Recommended baseline environment:

```bash
cd /run/host/home/skink/projects/voloxide/Voloxide
source /opt/ros/jazzy/setup.zsh
source /home/skink/projects/rosflight_setup/workspace/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/run/host/home/skink/projects/voloxide/Voloxide/target/ros2/roslog
```

## Correct Startup Sequence

Sources checked:

- ROSflight standalone sim tutorial:
  `https://docs.rosflight.org/latest/user-guide/tutorials/setting-up-rosflight-sim/`
- ROSflight manual flight tutorial:
  `https://docs.rosflight.org/latest/user-guide/tutorials/manually-flying-rosflight-sim/`
- ROScopter in sim tutorial:
  `https://docs.rosflight.org/latest/user-guide/tutorials/setting-up-roscopter-in-sim/`
- Local C/ROS2 code in `/home/skink/projects/rosflight_setup/workspace/src`.

The tutorial-level sequence is:

1. Source ROS2 and the ROSflight workspace.
2. Launch the multirotor standalone simulator:

   ```bash
   ros2 launch rosflight_sim multirotor_standalone.launch.py
   ```

   Add `use_vimfly:=true` only for manual keyboard control. In automated tests we leave VimFly off
   so the `rc.py` node exposes `/toggle_arm` and `/toggle_override`.

3. Launch the ROScopter autonomy stack and visualization helpers.
4. Load the mission:

   ```bash
   ros2 service call /path_planner/load_mission_from_file rosflight_msgs/srv/ParamFile \
     "{filename: /home/skink/projects/rosflight_setup/workspace/install/roscopter/share/roscopter/params/multirotor_mission.yaml}"
   ```

5. Arm and disable RC override:

   ```bash
   ros2 service call /toggle_arm std_srvs/srv/Trigger
   ros2 service call /toggle_override std_srvs/srv/Trigger
   ```

6. Monitor `/estimated_state`, `/high_level_command`, `/command`, `/status`, `/sim/pwm_output`, and
   `/sim/truth_state`, with rosbag capture for the C baseline trajectory.

The C-code-derived preconditions are stricter than the tutorial text:

- The manual-flight tutorial confirms parameters and IMU calibration are mandatory before arming.
  It also documents `multirotor_init_firmware.launch.py` as the convenience path for parameter load,
  IMU calibration, and parameter write.
- `rosflight_sim/launch/multirotor_init_firmware.launch.py` loads
  `params/multirotor_firmware/multirotor_combined.yaml`, calls `/calibrate_imu`, then sleeps before
  `/param_write`.
- `rosflight_io` exposes `/param_load_from_file` and `/calibrate_imu`.
- `rosflight_firmware/src/sensors.cpp` sets `ERROR_UNCALIBRATED_IMU` when all accel/gyro bias params
  are zero. Calibration only completes after more than 1000 IMU samples and then clears that error.
- `rosflight_firmware/src/state_manager.cpp` refuses arming while `ERROR_UNCALIBRATED_IMU` is set.

Therefore the harness startup sequence must include this firmware init gate before arm:

1. Wait for the sim, `sil_board`, and `rosflight_io` services/topics.
2. Call `/param_load_from_file` with the multirotor firmware parameter file.
3. Wait for `/all_params_received`.
4. Call `/calibrate_imu`.
5. Wait until accel/gyro bias params become nonzero or the uncalibrated-IMU error has cleared.
6. Arm with `/toggle_arm`.
7. Disable RC override with `/toggle_override` for offboard/autonomy tests.
8. Keep publishing `/command` or let ROScopter publish `/command`; offboard will not stay active
   without a current command stream.

Manual-flight note:

- The `/toggle_arm` and `/toggle_override` services are available from `rc.py` only when using the
  simulated joystick path. If VimFly or a supported transmitter is active, the tutorial says to arm
  and toggle override through that input device instead. VimFly maps `t` to arm/disarm and `r` to RC
  override.

Local launch-file discrepancy:

- The tutorial uses `ros2 launch roscopter_sim sim.launch.py`.
- Local `roscopter_sim/launch/sim.launch.py` includes `roscopter.launch.py` but does not forward the
  `state_topic` launch argument.
- Local `roscopter/launch/roscopter.launch.py` does support `state_topic`, defaulting to
  `estimated_state`.
- For deterministic standalone acceptance tests, the harness launches `roscopter.launch.py`
  directly with `state_topic:=sim/roscopter/state`, then starts the same helper nodes:
  `rviz_waypoint_publisher` and `sim_state_transcriber`. Current ROS2 `sim_state_transcriber`
  publishes the converted ROScopter state on `sim/roscopter/state`; using the older-looking
  `truth/NED` name leaves the trajectory follower without state input.

## Files Rebuilt

Removed the previous broken scripts and launch files:

- `scripts/sim_directional_acceptance.py`
- `scripts/sim_roscopter_waypoint_acceptance.py`
- `scripts/inspect_waypoint_bag.py`
- `scripts/summarize_waypoint_bag.py`
- `ros2/voloxide_sil_board_shim/launch/multirotor_standalone_upstream_baseline.launch.py`
- `ros2/voloxide_sil_board_shim/launch/multirotor_standalone_voloxide.launch.py`

Added a fresh harness:

- `scripts/sil_test_lib.py`
- `scripts/c_firmware_arming_acceptance.py`
- `scripts/c_firmware_joystick_modes_acceptance.py`
- `scripts/c_firmware_passthrough_acceptance.py`
- `scripts/c_firmware_waypoint_acceptance.py`
- `scripts/run_c_firmware_acceptance_suite.py`
- `ros2/voloxide_sil_board_shim/launch/multirotor_standalone_sil.launch.py`

## Current Results

Passing:

- `python3 -m py_compile scripts/*.py`
- `python3 scripts/c_firmware_arming_acceptance.py --no-rviz`
  - C `sil_board` and `rosflight_io` connect over UDP.
  - Firmware parameters load.
  - `/calibrate_imu` eventually produces nonzero accel/gyro bias params.
  - `/toggle_arm` produces `Autopilot ARMED`.
  - `/toggle_override` plus a neutral `/command` produces an observed `offboard=true` status.
- `python3 scripts/c_firmware_waypoint_acceptance.py --no-rviz --no-rosbag --per-waypoint-timeout 90`
  with an isolated `ROS_DOMAIN_ID=77`.
  - The C baseline completed the five-point ROScopter tutorial mission headless.
  - Output artifacts:
    - `target/waypoint_paths/c-firmware-waypoints-20260519.csv`
    - `target/waypoint_paths/c-firmware-waypoints-20260519-waypoint-visits.csv`
    - `target/waypoint_paths/c-firmware-waypoints-20260519.png`
  - Accepted waypoint samples:
    - `(0.0, 0.0, -10.0)` reached approximately `(0.00, 0.00, -6.00)`
    - `(20.0, 0.0, -10.0)` reached approximately `(16.07, 0.00, -10.72)`
    - `(20.0, -20.0, -20.0)` reached approximately `(20.11, -16.22, -18.72)`
    - `(0.0, -20.0, -20.0)` reached approximately `(3.95, -20.11, -20.59)`
    - `(0.0, 0.0, -40.0)` reached approximately `(-0.02, -3.11, -37.48)`

Failing or incomplete:

- `c_firmware_joystick_modes_acceptance.py`
  - Initial version over-drove the sim and later truth-state samples became invalid.
  - The script now lowers throttle and resets `/dynamics/set_sim_state` between cases.
  - Latest rerun exposed a script typo, now fixed, but the corrected test still needs rerun.
- `c_firmware_passthrough_acceptance.py`
  - The stack starts and initializes, but `/sim/pwm_output` did not show a response to direct
    `/command` publication in the latest run.
  - Hypothesis: the test may still be masking offboard output with RC override state or checking the
    wrong observable. Next step is to rosbag `/status`, `/command`, `/sim/pwm_output`, and
    `/sim/truth_state` during the passthrough run and confirm whether firmware reports offboard
    active while the command stream is present.
- `c_firmware_waypoint_acceptance.py`
  - Headless waypoint completion now works, but RViz/rosbag visual runs remain unstable on this
    machine/session.
  - Visual runs have shown jitter, repeated `rosflight_sil_manager` service timeout warnings, and
    eventual NaN transforms in RViz. The current working assumption is environment/timing
    instability rather than a proven C firmware logic problem.
  - We are going to preserve this handoff state, push it to `main`, and then reset the distrobox and
    computer before re-downloading/rebuilding everything and retrying the C visual baseline.

## Scientific Method Notes

### Failure 1: ROS2 socket errors

Observation: Fast DDS and Boost ASIO failed with `Operation not permitted` in the sandbox.

Hypothesis: the ROS2/MAVLink C baseline is structurally runnable, but the sandbox blocks required
network interface and UDP socket operations.

Test: rerun the C arming script outside the sandbox with Fast DDS.

Result: C `sil_board` and `rosflight_io` connected successfully over UDP.

Conclusion: C baseline tests must be run outside the sandbox.

### Failure 2: IMU calibration gate

Observation: `/calibrate_imu` returns success immediately, but arming can still fail with
`Unable to arm: IMU not calibrated`.

Hypothesis: the service only queues calibration; the test must wait for firmware to collect enough
IMU samples and publish nonzero bias params.

Test: wait for any of `ACC_*_BIAS` or `GYRO_*_BIAS` to become nonzero before arming.

Result: arming acceptance passed after the wait was increased.

Conclusion: all C baseline tests must treat calibration as asynchronous.

### Failure 3: Passthrough no PWM response

Observation: direct `/command` publication produced no observed PWM delta in the latest run.

Hypotheses:

1. RC override is still active and masking offboard command.
2. `/status.offboard` is not active during the command stream.
3. `/sim/pwm_output` is the wrong immediate assertion for this command mode or timing.
4. The command values are too conservative or not mapped as expected.

Next test:

Record a rosbag during passthrough with `/status`, `/command`, `/sim/pwm_output`, and
`/sim/truth_state`, then inspect whether command messages are present, whether `offboard=true` is
reported, and whether PWM is published at all.

## Next Steps

1. Push the current recovery work and generated C baseline artifacts to `main`.
2. Reset the distrobox/computer, then re-download and rebuild the ROSflight/ROScopter/Voloxide
   environment from scratch.
3. Re-run the C visual waypoint baseline first. The C code should work cleanly in a fresh
   environment before moving on.
4. Rerun the corrected joystick and passthrough tests.
5. Only after the C baseline is verified, build and test `firmware:=voloxide` through the FFI shim.
