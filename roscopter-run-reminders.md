# ROScopter Run Reminders

Local notes from the July 1, 2026 Veloxity ROScopter sim run.

## Environment

- This checkout is under `/home/skink/projects/ROSflight_ubuntu22/.distrobox-home/ROSflight_ubuntu22`, not the older path shown in `docs/tutorials/veloxity-roscopter-sim-end-to-end.md`.
- Source order used for ROS commands:
  1. `/opt/ros/humble/setup.zsh`
  2. `rosflight/rosflight/workspace/install/setup.zsh`
  3. `Veloxity/workspace/install/setup.zsh`

## Things To Remember

- `scripts/build_and_source_ros2_shim.zsh` builds successfully after sourcing ROS 2 and the ROSflight workspace.
- ROS 2 launch/discovery needs normal DDS network access in this environment. The first sandboxed launch failed with CycloneDDS `failed to enumerate interfaces for "udp"`.
- RViz also failed inside the sandbox because it could not connect to display `:0`. The escalated launch with `use_rviz:=true` started successfully.
- For runs the user needs to see, launch Veloxity SIL with `use_rviz:=true`; the successful visual launch showed RViz startup, OpenGL initialization, `veloxity_sil_board ready`, and `rosflight_io` heartbeat connection.
- Expected simulator startup lines seen:
  - `veloxity_sil_board ready: service=sil_board/run, pwm=sim/pwm_output`
  - `rosflight_io: Connecting over UDP to "localhost:14525", from "localhost:14520"`
  - `rosflight_io: Got HEARTBEAT, connected.`
  - `rosflight_io: Received all parameters`
- `rosflight_io` reports a version warning: ROSflight `2.0`, firmware `eloxity 1.0`. This did not stop startup.
- Initial `rosflight_io` errors before firmware init are expected: RC override active and uncalibrated IMU.
- The firmware init launch can return service success even when the autopilot reports calibration failure in the simulator log. Check the launch session output after init.
- On this run, `/imu/data` and `/sim/pwm_output` both measured about `400 Hz`, but `/sim/truth_state` showed the vehicle moving/rotating after init; that caused `Gyro calibration failed` and `Accelerometer calibration failed: too much movemen`.
- Do not proceed to ROScopter autonomy until the sim has been reset and IMU calibration has been verified from logs/status.
- Recovery sequence that worked:
  1. Call `/dynamics/set_sim_state`.
  2. Verify `/sim/truth_state` is stationary.
  3. Call `/calibrate_imu`.
  4. Confirm launch log contains `Gyro Calibration complete!`, `Accelerometer Calibration Complete!`, and `Autopilot RECOVERED ERROR: Uncalibrated IMU`.
  5. Call `/calibrate_baro` and `/param_write`.
  6. Verify `/status` has `error_code: 0`.
- After recovery, `/baro` measured about `100 Hz`.
- ROScopter launch pre-mission checks:
  - Nodes appeared: `/controller`, `/estimator`, `/trajectory_follower`, `/path_manager`, `/path_planner`, `/roscopter_truth`.
  - `/estimated_state`, `/high_level_command`, and `/command` measured about `390 Hz`.
- Mission load returned `success=True`; `/waypoints` published the first default waypoint `[0.0, 0.0, -10.0]`, and the path planner logged five waypoints from `multirotor_mission.yaml`.
- After `/toggle_arm`, status showed `armed: true`, but `rc_override` was still nonzero. The docs' `/toggle_override` step was required.
- After `/toggle_override`, status showed `armed: true`, `rc_override: 0`, `offboard: true`, `error_code: 0`; PWM rose to about `1505-1508` on the four motor outputs and `/sim/truth_state` showed the vehicle airborne and moving along the mission.
- During arm/override transitions, `rosflight_io` reported brief `Unhealthy estimator` errors and then `Autopilot RECOVERED ERROR: Unhealthy estimator`. This matches the tutorial caveat for mode transitions.
- Final verification from this run:
  - `/status`: `armed: true`, `failsafe: false`, `rc_override: 0`, `offboard: true`, `error_code: 0`.
  - `/command`: active offboard command with nonzero vertical command.
  - `/sim/pwm_output`: four motor outputs around `1505-1508`.
  - `/sim/truth_state`: about `400 Hz`; vehicle had progressed to around `[19.5, -19.4, -23.5]`, consistent with moving through the loaded mission.

## Default ROScopter Launch Recipe

Use this as the default when asked to bring up ROScopter with the Veloxity SIL visualizer and waypoints.

1. Source ROS in this order:
   - `/opt/ros/humble/setup.zsh`
   - `rosflight/rosflight/workspace/install/setup.zsh`
   - `Veloxity/workspace/install/setup.zsh`
2. Launch Veloxity SIL with RViz enabled:
   - `ros2 launch veloxity_sil_board_shim multirotor_standalone_sil.launch.py use_rviz:=true`
3. Reset the vehicle before calibration:
   - Call `/dynamics/set_sim_state`.
   - Verify `/sim/truth_state` is stationary at the origin.
4. Run documented firmware init:
   - `ros2 launch veloxity_sil_board_shim veloxity_multirotor_init_firmware.launch.py write_delay:=3.0`
   - Confirm logs show gyro and accelerometer calibration complete, baro calibration complete, and param write success.
   - Verify `/status` has `armed=false`, `failsafe=false`, `error_code=0`.
5. Keep the recommended/custom autonomy mixer:
   - Verify `PRIMARY_MIXER=11.0`.
   - Verify `USE_MOTOR_PARAM=1.0`.
   - Do not switch to canned quad-X for this default autonomy path.
6. Launch ROScopter:
   - `ros2 launch roscopter_sim sim.launch.py`
   - Verify `/estimated_state`, `/high_level_command`, and `/command` are publishing near `390 Hz`.
   - Verify `/estimated_state.p_d` and `/sim/truth_state.pose.position.z` are close to ground before arming.
7. Load the waypoint mission:
   - Call `/path_planner/load_mission_from_file` with `roscopter/params/multirotor_mission.yaml`.
   - Verify `/waypoints` publishes the first waypoint `[0.0, 0.0, -10.0]`.
8. Record before arm/release if doing a capture:
   - Include `/sim/truth_state`, `/estimated_state`, `/trajectory_command`, `/high_level_command`, `/command`, `/status`, `/sim/pwm_output`, and `/waypoints`.
9. Arm and hold under override:
   - Call `/toggle_arm`.
   - Verify `armed=true`, `rc_override` nonzero, `error_code=0`, PWM idle around `1100`, and truth still stationary.
   - Let the estimator settle briefly; this helped remove the earlier multi-meter altitude bias.
10. Release override:
   - Call `/toggle_override`.
   - Verify `rc_override=0`, `offboard=true`, `error_code=0`, and PWM is controlled, not saturated.
   - Watch early takeoff for lateral transient and verify estimator/truth altitude agreement.

Default run expectations with the calibrated/recommended setup:

- Baro/altitude alignment should start near ground, not several meters high.
- Takeoff may still show a lateral component; the final capture saw about `0.281` rad roll and `-0.315` rad pitch at `t=0.2 s`.
- After transition to passthrough with the recommended motor-parameter mixer, PWM should remain controlled rather than saturating.
- The visual should be available in RViz because SIL was launched with `use_rviz:=true`.

## July 1 Follow-Up: Altitude Bias

- Holding armed with RC override active before releasing the mission lets ROScopter's estimator collect its own barometer baseline while the vehicle is stationary.
- That alone improved ground alignment, but early climb still showed a scale error when estimator `rho` was hard-set to `1.225`.
- Setting `/estimator.rho` to `0.0` in `roscopter/params/estimator.yaml` lets ROScopter compute air density from initial GPS altitude instead of forcing sea-level density.
- With `rho: 0.0`, ground truth and estimate lined up before release (`truth z ~= 0`, `p_d ~= -0.04`), and the first useful post-release sample had `truth z = -11.85`, `estimated p_d = -11.94`.
- Next issue to investigate: right after takeoff, the vehicle initially moves laterally in a nearly linear direction before approaching the goal. Capture `/sim/truth_state`, `/estimated_state`, `/trajectory_command`, `/high_level_command`, `/command`, and `/status` during the first 10-15 seconds after `/toggle_override`.
- Do not interpret the later sample after re-enabling override mid-flight as normal barometer drift: it showed estimator failure (`truth z ~= 0.05`, `/estimated_state.p_d ~= 256`, status had shown `error_code: 8`). That run was disarmed and should be reset before more conclusions.

## July 1 Follow-Up: Takeoff Lateral Transient

- Bag used for analysis: `Veloxity/takeoff_logs/takeoff_transient_20260701`.
- In that bag, release is the first `/status` sample with `armed=true` and `rc_override=0`.
- The trajectory command stayed essentially vertical at `[0, 0, -10]` until about 4 s after release, so the early lateral motion is not the waypoint/path manager starting the mission early.
- ROScopter takeoff uses its state-machine takeoff controller first, not the normal trajectory passthrough. It holds `takeoff_n_pos_`, `takeoff_e_pos_`, and `takeoff_yaw_`, and commands `takeoff_d_vel`.
- `/command.mode=2` during takeoff is ROSflight `MODE_ROLL_PITCH_YAWRATE_THROTTLE`; `u[3]` is roll angle, `u[4]` is pitch angle, `u[5]` is yaw rate, and `u[2]` is throttle.
- `u[3]` and `u[4]` are radians. Do not read `-1.04` as degrees; it is about `-60 deg`.
- `ControllerCascadingPID::pass_to_firmware_controller` zeros roll/pitch/yaw-rate while `abs(xhat_.p_d) < min_altitude_for_attitude_ctrl`. Active value was `0.3 m`.
- At `t=0.221 s`, estimated `p_d` crossed about `-0.30 m` and `/command.u[3]` jumped from `0.0` to about `-1.04 rad`; the vehicle immediately rolled and translated laterally.
- The code uses `max_roll_deg` and `max_pitch_deg` directly as radian limits in `facc_racc_dacc_yawrate()` and `pass_to_firmware_controller()`. With the active value `25.0`, the intended 25 deg cap is effectively a 25 rad cap.
- The first fix to try is converting degree limits to radians in those paths, then recapturing the first 5 s after release. Also watch for derivative kick from the position/velocity cascade when attitude control is first unmasked.

## July 1 Follow-Up: Quad-X Isolation Test

- Bag used for the quad-X takeoff/hold test: `Veloxity/takeoff_logs/quadx_takeoff_hold_20260701`.
- The test changed runtime firmware params only: `PRIMARY_MIXER=2.0` and `USE_MOTOR_PARAM=0.0`. No source changes were made for this test.
- Pre-release checks were healthy: `armed=true`, `failsafe=false`, `offboard=true`, `rc_override=3`, four motor outputs at idle, and truth/estimate near the origin.
- After `/toggle_override`, the vehicle climbed uncontrollably. At release, the command was still takeoff mode with normal throttle (`mode=2`, `u[2] ~= 0.74`, roll/pitch zero), but after the ROScopter switch to passthrough the command became `mode=0` with `u[2] ~= -44.9`.
- Firmware/Veloxity logged repeated `Output from mixer is ... Check mixer` warnings and all four motor PWMs saturated near `2000`.
- This is not the same signature as the original lateral takeoff transient. Quad-X passthrough is incompatible with the current ROScopter force-command units because the canned non-motor-parameter mixer treats the passthrough force as a normalized mixer input.
- Do not use canned quad-X for definitive waypoint captures unless the passthrough thrust units are changed/matched. Reinitialize firmware back to the documented custom mixer/motor-parameter configuration before final waypoint runs.

## July 1 Follow-Up: Angle/Rate Autonomy Modes

- ROScopter message modes support firmware-controller command paths:
  - `MODE_ROLL_PITCH_YAWRATE_THROTTLE = 6`
  - `MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 7`
- Those map to ROSflight command modes:
  - `MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2`
  - `MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1`
- The default trajectory follower does not currently use those modes for waypoint flight. It hard-codes `MODE_ROLL_PITCH_YAWRATE_THRUST_TO_MIXER`, which then becomes mixer passthrough.
- Takeoff already uses the firmware angle/throttle path through `MODE_ROLL_PITCH_YAWRATE_THROTTLE`; that is why early takeoff commands appear as `/command.mode=2`.
- To test quad-X with firmware angle/rate handling during autonomy, add a local experimental option that converts trajectory follower output to angle/rate plus normalized throttle, or bypass ROScopter controller and publish `rosflight_msgs/msg/Command` directly to the offboard command topic.
- Do not confuse ROScopter mode `4` with firmware mode `4`: ROScopter mode `4` is `MODE_NPOS_EPOS_DVEL_YAW`, a high-level input to the cascading controller, not a ROSflight firmware command mode.

## July 1 Temporary Quad-X Angle-Mode Experiments

- Example folder: `Veloxity/examples/quadx_angle_waypoints`.
- Repeatable harness files:
  - `run_waypoint_angle_experiment.zsh` launches the experiment, records a bag, and restores the recommended mixer on cleanup.
  - `trajectory_to_angle_command.py` is the experimental final-stage controller.
  - `angle_waypoint_baseline.yaml` contains the tunable gains/limits for the final-stage controller.
  - `analyze_waypoint_angle_bag.py` summarizes command modes, command ranges, PWM saturation, tracking error, altitude error, and estimator-vs-truth error.
  - `README.md` has the visual SIL, firmware init, run, and analyze commands.
- Direct hold script: `quadx_angle_hold.py`.
  - Publishes `rosflight_msgs/msg/Command` directly on `/command`.
  - Uses firmware `MODE_ROLL_PITCH_YAWRATE_THROTTLE` (`/command.mode=2`) with canned quad-X (`PRIMARY_MIXER=2`, `USE_MOTOR_PARAM=0`).
  - Altitude hold came from the temporary companion node adjusting normalized throttle `u[2]`; firmware handled roll/pitch/yaw-rate attitude control.
  - Bag: `Veloxity/takeoff_logs/quadx_angle_mode_20260701`.
  - Result: successful stable hover around `3.79 m`, with centimeters of lateral drift and PWM around `1477-1482`.
- Waypoint-angle script: `trajectory_to_angle_command.py`.
  - Runs ROScopter estimator, path manager, and path planner, but not the stock ROScopter controller or trajectory follower.
  - Consumes `/trajectory_command` plus `/estimated_state`, then publishes ROSflight `/command.mode=2` with normalized throttle.
  - First waypoint attempt without target slewing was unstable. Bag: `Veloxity/takeoff_logs/quadx_waypoint_angle_mode_20260701`.
  - Failure cause: path-manager trajectory time advanced while RC override was active, so release caused an immediate jump to a far-ahead trajectory target. Stock ROScopter hides this with its takeoff/position-hold state machine; the temporary node did not.
  - Patched behavior: while disarmed or `rc_override != 0`, freeze the filtered target at current estimated state. After release, slew the filtered target toward `/trajectory_command`.
- Successful bag: `Veloxity/takeoff_logs/quadx_waypoint_angle_mode_slew_20260701`.
- Successful result: quad-X flew waypoints with `/command.mode=2`, controlled PWM, and reached the mission path through approximately `(20, 0, -10)` and `(20, -20, -20)`, then started the next leg.
- Representative sample from the successful run: truth near `(18.98, -18.06, -21.19)`, estimator near `(17.92, -19.91, -20.71)`, trajectory command near `(17.73, -20.0, -20.0)`.
- Analyzer output for the successful run:
  - All post-release `/command` samples were firmware angle mode: `{2: 5780}`.
  - Command ranges: throttle `[0.581, 0.767]`, max roll `0.226 rad`, max pitch `0.174 rad`, max yaw-rate `0.694 rad/s`.
  - PWM range was `[1100, 2000]` with `13` saturation samples.
  - Lateral tracking error versus `/trajectory_command`: mean `3.02 m`, p95 `6.11 m`, max `6.93 m`.
  - Altitude error `truth.z - trajectory.z`: mean `0.79 m`, p95 absolute `7.49 m`, max absolute `10.00 m`.
  - Estimator altitude was close to truth: mean `0.40 m`, p95 absolute `0.59 m`, max absolute `0.60 m`.
- Comparison target: the recommended mixer/passthrough run was more stable through waypoint transitions. Use the harness to vary `angle_waypoint_baseline.yaml` and compare analyzer metrics, especially lateral p95/max, altitude p95/max, and PWM saturation samples.
- Cleanup after these tests:
  - Stop temporary Python node and ROScopter estimator/path nodes before resetting.
  - Re-enable override, disarm, then reset dynamics.
  - Restore recommended defaults afterward: `PRIMARY_MIXER=11`, `USE_MOTOR_PARAM=1`.

## July 1 Quad-X Angle-Mode Controller Tuning

- Control-theory direction: preserve ROScopter/Veloxity path-planner output. The temporary final-stage controller should track `/trajectory_command.position`, use `/trajectory_command.velocity` as desired velocity, and use `/trajectory_command.acceleration` as feedforward, then convert the resulting desired acceleration to roll/pitch plus normalized throttle. Do not replace the path planner with a crude waypoint slew unless explicitly testing a safety fallback.
- `trajectory_to_angle_command.py` was updated with `use_filtered_target: false` as the default path-planner-feedforward mode. It still has position-error limiters:
  - `max_horizontal_position_error_m: 3.0`
  - `max_vertical_position_error_m: 2.0`
- Current tuned parameters in `angle_waypoint_baseline.yaml`:
  - `kp_n/e: 0.45`, `kd_n/e: 0.70`, `max_horizontal_accel_m_s2: 2.6`
  - `kp_d_throttle: 0.035`, `kd_d_throttle: 0.075`, `kff_d_accel_throttle: -0.010`
  - `max_angle_rad: 0.30`, `max_yaw_rate_rad_s: 0.70`
- Bad aggressive run: `Veloxity/takeoff_logs/quadx_waypoint_angle_mode_tuned1b_20260701`.
  - Lateral tracking improved, but vertical control was unacceptable.
  - Analyzer: throttle hit both limits `[0.430, 0.820]`; roll/pitch hit `0.300 rad`; PWM saturation samples `2657`; lateral mean `1.31 m`, p95 `2.71 m`; altitude p95 absolute `19.16 m`, max absolute `21.75 m`.
  - Lesson: do not raise vertical throttle gains and vertical slew aggressively. It makes the vehicle faster laterally but causes large altitude excursions and throttle saturation.
- Best run from this iteration: `Veloxity/takeoff_logs/quadx_waypoint_angle_mode_planner_ff_clean_20260701`.
  - This used the path-planner feedforward controller directly, with bounded position error.
  - Analyzer: `/command.mode=2` for all post-release samples; throttle `[0.633, 0.820]`; max roll `0.128 rad`; max pitch `0.186 rad`; max yaw-rate `0.215 rad/s`; PWM saturation samples `817`; lateral mean `1.09 m`, p95 `2.49 m`, max `2.78 m`; altitude mean `0.37 m`, p95 absolute `6.27 m`, max absolute `10.71 m`.
  - Live behavior: clean takeoff to `(0,0,-10)`, smooth first leg to `(20,0,-10)`, and successful transition/reach near `(20,-20,-20)` with much better lateral tracking than the slewed-target baseline.
- Invalid run to ignore for performance comparison: `Veloxity/takeoff_logs/quadx_waypoint_angle_mode_planner_ff_clean2_20260701`.
  - It was interrupted after release because the estimator/path-manager state was bad before flight; `/trajectory_command` jumped directly toward `(20,-20,-20)` at mission load.
  - Lesson: between repeat runs, reinitialize/calibrate firmware and estimator/baro before launching the experiment. A sim-state reset alone is not enough after a previous flight.
- Harness fixes:
  - `run_waypoint_angle_experiment.zsh` now refuses to start when stale estimator/path/temp-controller/bag processes are still running.
  - Cleanup no longer uses zsh's read-only `status` variable.
  - The runner forces known arm/override transitions instead of blindly toggling.
- For the next repeatable clean run:
  - First run `ros2 launch veloxity_sil_board_shim veloxity_multirotor_init_firmware.launch.py write_delay:=3.0`.
  - Verify `/estimated_state.p_d` is near ground before arming/release.
  - If `/trajectory_command` is already far past the first waypoint before release, abort and reset/calibrate; do not treat the bag as a control result.
- Full completed reference run: `Veloxity/takeoff_logs/quadx_waypoint_angle_mode_full_20260701`.
  - This run started with no stale estimator/path nodes and `/estimated_state.p_d = 0.0` during preflight.
  - It completed the waypoint sequence through `(0,0,-10)`, `(20,0,-10)`, `(20,-20,-20)`, `(0,-20,-20)`, and `(0,0,-40)`, then began another pass before the timed stop.
  - Analyzer: all post-release commands used firmware angle mode `{2: 12025}`; throttle `[0.662, 0.820]`; max roll `0.095 rad`; max pitch `0.110 rad`; max yaw-rate `0.502 rad/s`; PWM saturation samples `7`.
  - Tracking: lateral mean `0.98 m`, p95 `2.58 m`, max `2.87 m`; altitude mean `0.92 m`, p95 absolute `1.96 m`, max absolute `2.34 m`; estimator altitude p95 absolute `0.69 m`.

## July 1 Final Recommended-Mixer Waypoint Capture

- Final waypoint bag: `Veloxity/takeoff_logs/recommended_waypoints_20260701`.
- Visual SIL/RViz was launched with `use_rviz:=true` and stayed up for the run.
- Firmware was reinitialized after the quad-X test. Verified recommended params before flight: `PRIMARY_MIXER=11.0`, `USE_MOTOR_PARAM=1.0`.
- Pre-release checks:
  - `/status`: `armed=true`, `failsafe=false`, `rc_override=3`, `offboard=true`, `error_code=0`.
  - PWM idle: first four motor outputs at `1100`.
  - Truth at ground/origin.
  - Estimator altitude near ground: `p_d` about `-0.13 m` to `-0.21 m`, not the earlier multi-meter bias.
- After release:
  - Status was healthy with `rc_override=0`.
  - Immediate PWM was normal, about `1487-1489`, not saturated.
  - Takeoff still showed a lateral component: at `t=0.2 s`, command roll/pitch were about `0.281` and `-0.315` rad, and by `t=1.0 s` truth was about `(0.40, 0.32, -1.77)`.
  - At `t=2.0 s`, ROScopter had switched to passthrough (`mode=0`) with custom motor-parameter mixing, and PWM stayed controlled around `1588-1630`.
  - The vehicle tracked the mission: near `(20.37, -3.07, -12.11)` around `t=20 s`, `(10.06, -19.78, -20.33)` around `t=40 s`, and `(0.70, -10.76, -29.16)` around `t=60 s`.
  - Estimator altitude remained close to truth during the sampled mission points, usually within roughly `0.2-0.5 m` in these samples.
