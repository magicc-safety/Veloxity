# ROSplane startup command development log

Last updated: 2026-07-22 (selectable C/Veloxity firmware and verified loading)

## Current live session

A persistent Screen session named `veloxity-plane` is currently running for the
user to inspect. Do not start another simulator or another GCS launch on the
same ROS domain while this session is active.

Current state at the latest checkpoint:

- The ROSplane documentation hourglass mission loaded successfully.
- The simulator was started with the new default backend, Veloxity, with its
  normal RViz visualization enabled.
- ROSflight reports `armed: true`, `rc_override: 0`, `offboard: true`, no
  failsafe, and zero errors.
- The visible clean run observed motion near NED position
  `(21.5, -38.2, -8.7)` after autonomy was released.
- The waypoint visualization uses `/rosplane_waypoint_publisher` with the
  simulator's original `/rviz`; no second RViz process is needed.
- There are no duplicate node names and only the simulator's two static TF
  publishers.
- `/rviz/waypoint` contains active sphere, text, and line-strip markers at the
  mission coordinates, and the simulator RViz configuration is subscribed to
  that topic with transient-local durability.

The user can inspect the terminal session with:

```zsh
screen -r veloxity-plane
```

To shut this particular live run down safely, first use the `commands` window:

```zsh
p_toggle_sim_override  # take control back from ROSplane
p_toggle_sim_arm       # disarm; verify the response says disabled
p_show_status          # confirm armed: false
```

Then detach if necessary and remove the session with:

```zsh
screen -S veloxity-plane -X quit
```

## Project boundaries

- The fixed-wing configuration now lives under
  `~/.config/veloxity/airframes/fixedwing`, parallel to `3dquad`, with its own
  setup, commands, firmware YAML, mission, snapshot directories, and parameter
  assurance tools. `~/.config/veloxity/commands_plane` is only a compatibility
  entry point.
- Integration-layer files under `sim/ros2/veloxity_sil_board_shim` may be
  changed. Do not modify Veloxity firmware.
- Do not modify ROScopter, any `v_*` startup file, or its helper scripts. They
  may be inspected read-only and their patterns copied into plane-specific
  integration code.
- Do not modify any ROSflight, `rosflight_io`, ROSplane, generated, installed,
  build, source, launch, parameter, or package file.
- `rosflight_io` must remain completely unmodified.
- ROSflight and ROSplane workspaces are runtime and read-only reference
  dependencies. They may be inspected, sourced by the caller, and run for
  testing.
- Do not add commands that source helper scripts from outside Veloxity. Assume
  ROS 2 and the ROSflight workspace have already been sourced by the caller.
- Prefer the existing upstream `rosflight_sim` standalone fixed-wing simulator.
- Testing must not leave background ROS processes or screen sessions behind.

## User requests and decisions

- The user likes the current quadcopter workflow: one shell function per stack
  component plus a GNU Screen helper that creates named interactive terminals.
- The plane command file must be independent of the working quadcopter setup.
- The plane functions should use a distinct `p_` prefix so sourcing the new file
  alongside the existing `v_` quadcopter functions does not overwrite them.
- This file is the durable handoff record and must be updated periodically so
  work can resume after a disconnect.

## Verified local environment

- Active home:
  `/home/skink/projects/ROSflight_ubuntu22/.distrobox-home/ROSflight_ubuntu22`
- Existing quad commands:
  `~/.config/veloxity/airframes/3dquad/commands.zsh`
- Existing quad setup:
  `~/.config/veloxity/airframes/3dquad/setup.zsh`
- `~/.zshrc.local` sources the ROS 2, ROSflight, and Veloxity install
  environments, then sources the quad setup and command files.
- GNU Screen 4.09.00 is installed.
- Available relevant packages include `rosflight_io`, `rosflight_sim`,
  `rosplane`, `rosplane_extra`, `rosplane_gcs`, `rosplane_sim`, and
  `rosplane_tuning`.
- Available ROSplane executables:
  `controller`, `estimator`, `path_follower`, `path_manager`, and
  `path_planner`.
- `rosplane_sim` provides `sim_state_transcriber`.
- `rosflight_sim` provides the standalone dynamics, sensors, fixed-wing forces
  and moments, upstream C SIL manager/board, RC input, and visualization nodes.
- The command file is installed at
  `~/.config/veloxity/airframes/fixedwing/commands.zsh` and passes `zsh -n`.

## Architecture and current direction

The plane workflow will have two layers:

1. `veloxity_sil_board_shim fixedwing_standalone_sil.launch.py` starts the
   upstream fixed-wing physical simulation, unmodified `rosflight_io`, RC
   input, and RViz visualization. Its `firmware` launch argument selects the
   upstream C SIL endpoint or the Veloxity endpoint and defaults to Veloxity.
2. Individual `rosplane` commands start estimator, truth transcriber, path
   planner, path manager, path follower, and controller. Keeping these as
   separate commands matches the quadcopter workflow and makes failures easy to
   isolate.

Planned convenience commands also include firmware initialization, mission
loading/printing, status display, bag recording, an optional ROSplane GCS, a
help command, and a Screen session builder with one named window per component.

Implemented public commands:

- `p_start_screen`, `p_start_sim`, `p_load_firmware_params`,
  `p_save_firmware_snapshot`, and `p_load_firmware_snapshot`
- `p_start_estimator`, `p_start_truth`, `p_start_path_planner`,
  `p_start_path_manager`, `p_start_path_follower`, and `p_start_controller`
- `p_start_waypoint_viz`, its compatibility alias `p_start_gcs`,
  `p_load_mission`, `p_publish_next_waypoint`, `p_clear_waypoints`, and
  `p_print_waypoints`
- `p_calibrate_imu`, `p_calibrate_baro`, `p_show_status`, `p_start_bag`, and
  `p_help`
- `p_toggle_sim_arm` and `p_toggle_sim_override` for the no-joystick simulated
  RC fallback

The plane-specific copy of `verified_param_loader.py` preflights every parameter in the fixed-wing YAML,
sets mismatches one at a time, waits for acknowledgment-backed readback with
retries, and performs a final full-file verification. Only after PASS does
`p_load_firmware_params` publish the existing `status/params_changed`
notification so the fixed-wing force model reloads the confirmed mixer table.

`p_save_firmware_snapshot` does not use rosflight_io's file-writing service.
It derives the complete parameter name/type schema from the selected backend's
current source (`Veloxity/crates/veloxity_core/src/params.rs` or ROSflight
firmware `src/param.cpp`), waits for rosflight_io's complete-table signal,
reads every value individually through `/param_get`, writes and fsyncs its own
temporary YAML, parses it back, requires the exact source-defined name set,
rereads every firmware value, and atomically renames the verified result. This
assumes the checked-out parameter-definition source matches the running build.

The fixed-wing firmware directory names its reviewed backend startup subsets
explicitly: `firmware-startup-veloxity.yaml` and `firmware-startup-c.yaml`.
Their 16 shared entries have identical types and values. The only discrepancy
is the Veloxity-only `CHN_OUTPUT_MASK=0`, which keeps every physical output
disabled until explicitly enabled; it is omitted from the C file because the
upstream C firmware does not expose that parameter. The adjacent firmware
README records this distinction.

New interactive shells source both fixed-wing `setup.zsh` and `commands.zsh`
through `~/.zshrc.local`. The Screen helper creates interactive shells, so the
same `p_` functions are available in every window without running commands for
the user.

## Proven pitfalls and approaches not to repeat

- `/home/skink/.config/veloxity` is not the active configuration path inside
  this distrobox. Use `$HOME/.config/veloxity`.
- Do not assume `rosplane_sim sim.launch.py` starts aircraft physics. In this
  checkout it includes the ROSplane autonomy launch plus
  `sim_state_transcriber`; it does not start `rosflight_sim` fixed-wing
  dynamics or firmware.
- Do not pass `aircraft:=...` or `control_type:=...` to
  `rosplane.launch.py`. Although that launch file manually scans `sys.argv`, it
  does not declare those launch arguments, and the ROS 2 CLI rejects them as
  unrecognized.
- `ros2 launch ... --show-args` attempts to create a ROS log directory. In the
  restricted development environment, set `ROS_LOG_DIR` to a directory under
  `/tmp` for read-only launch inspection.
- The installed ROSplane share does not contain the example `missions`
  directory. A mission-loading helper should accept an explicit readable YAML
  path rather than relying on an installed default mission.
- Never start the full `rosplane_gcs rosplane_gcs.launch.py` on top of the
  standard fixed-wing standalone launch. Both launch an RViz node named
  `/rviz`, and both launch the world/NED static transforms. During testing this
  displayed the waypoints but polluted the ROS graph with a duplicate `/rviz`
  node and redundant TF publishers; the user correctly identified the graph
  corruption. The installed `p_start_waypoint_viz` replacement starts only the
  waypoint publisher and a uniquely named `/rosplane_waypoint_rviz`; it does
  not launch extra TF publishers. `p_start_gcs` is now only an alias to this
  safe replacement.
- ROSplane's waypoint services in this checkout are root-level relative names:
  `/add_waypoint`, `/clear_waypoints`, `/load_mission_from_file`,
  `/print_waypoints`, and `/publish_next_waypoint`. Do not copy the ROScopter
  convention of calling `/path_planner/...`; doing so waits forever even while
  the path planner node is healthy. The first draft made this assumption, the
  full-stack test exposed it, and the installed command file is corrected.
- Restricted sandbox execution cannot open the UDP/DDS sockets required by the
  simulator (`Operation not permitted`) or the X display required by RViz. A
  simulator failure with those exact errors is a test-sandbox limitation. The
  same command was rerun in the normal runtime context and succeeded.
- GNU Screen session creation also fails inside the restricted sandbox. It was
  successfully tested in the normal runtime context.
- With the simulated RC fallback, do not disable override before arming.
  ROSflight rejects that order with `RC throttle override must be active to
  arm`. On a fresh simulator, arm while override is initially enabled, confirm
  `armed: true`, and only then toggle override off to hand control to ROSplane.

## Test results

### Static and Screen tests

- The fixed-wing `setup.zsh`, `commands.zsh`, and compatibility entry point
  pass Zsh syntax checking.
- Installed ROSplane aircraft and estimator parameter files resolve and are
  readable.
- Every referenced package, executable, and launch file exists.
- `p_load_mission` rejects a missing argument with return code 2 and a useful
  usage message.
- `p_start_screen` creates windows named `firmware`, `estimator`, `truth`,
  `path_planner`, `path_manager`, `path_follower`, `controller`, `gcs`, and
  `commands`. Parameter loading and calibration are run explicitly from the
  `commands` window.
- A command injected into the `commands` window confirmed `p_start_sim` was a
  loaded shell function. The temporary session was removed.

### Simulator smoke test

The bounded upstream fixed-wing launch started all expected processes:

- two static transform publishers, RViz, and standalone visualization
- `rosflight_sil_manager`, `sil_board`, and unmodified `rosflight_io`
- standalone sensors, RC, fixed-wing forces/moments, and dynamics

`rosflight_io` connected over UDP to the upstream C SIL firmware endpoint,
received all parameters, and RViz initialized successfully. The test was stopped
with SIGINT after 15 seconds, and no related processes remained.

### Selectable firmware and verified Veloxity flight

`p_start_sim` now accepts `--firmware veloxity|c` and defaults to Veloxity.
The underlying fixed-wing launch declares the same argument and conditionally
starts exactly one endpoint:

- default/`veloxity`: `/veloxity_sil_board`
- `c`: upstream `/sil_board`

Both selector paths were smoke-tested with the same fixed-wing physics and
unmodified `rosflight_io`. The first full Veloxity attempt accepted commands,
armed, and produced nonzero PWM but remained stationary because the ROSFlight
force node made its one startup mixer request before `rosflight_io` had
discovered all 337 firmware parameters. A manual `status/params_changed=true`
immediately changed the wrench from zero to nonzero and started motion, proving
that firmware output and the mixer table were correct.

The permanent plane workflow now uses its own copy of ROScopter's verified
loader pattern. On the final clean run it reported:

```text
PASS: verified all 16 parameter(s) (0 changed, 16 already matched).
```

It then notified the force model, calibrated the IMU, and wrote parameters.
Without any manual diagnostic publication, default Veloxity armed under
override, transferred to computer control, remained failsafe-free/error-free,
produced a nonzero wrench, and moved through the simulation. The live validation
observed force approximately `(6.0, 3.5, -221.0) N` and truth position
approximately `(34.3, -19.1, -8.7) m` shortly after release.

### Full Screen stack test

The command file started the simulator and every individual ROSplane component.
The live ROS graph contained:

- `/controller`, `/estimator`, `/path_follower`, `/path_manager`, and
  `/path_planner`
- `/rosplane_truth`
- all expected ROSflight simulation, I/O, dynamics, sensor, visualization, and
  transform nodes

The control chain exposed `/estimated_state`, `/waypoint_path`,
`/controller_command`, and `/command`. Firmware initialization returned success
for fixed-wing parameter loading, simulated IMU calibration, and parameter
write.

After correcting the waypoint service namespace, loading the read-only example
mission by explicit source path returned `success=True`; the planner parsed and
printed all four waypoints. Every temporary Screen session and simulator process
was removed after testing.

### Autonomous waypoint flight without a physical controller

No physical controller is required. When no joystick is detected,
`rosflight_sim`'s `rc.py` publishes simulated RC input and provides
`/toggle_arm` and `/toggle_override`; the command file wraps them as
`p_toggle_sim_arm` and `p_toggle_sim_override`.

The four-waypoint example mission was flown successfully using the simulated RC
fallback. The verified fresh-simulator handoff was:

1. Start and initialize the full stack.
2. Load the mission.
3. Call `p_toggle_sim_arm` while override is still enabled.
4. Confirm ROSflight reports `armed: true`.
5. Call `p_toggle_sim_override` to release pilot override to ROSplane.

After 15 seconds, truth position was approximately
`(155.5, -158.1, -67.4)` NED at about `18.5 m/s`, headed toward the first
waypoint `(400, -250, -70)`. Thirty seconds later the aircraft was approximately
`(-338.9, -257.3, -74.7)` at `18.1 m/s`, and `/current_path` showed the mission
had advanced to the segment rooted at `(-400, -250, -70)`. ROSflight remained
armed, offboard, free of failsafe, and reported zero errors.

Shutdown was performed in the reverse safety order: enable override, toggle arm
off, confirm `armed: false`, then close the Screen session. No processes
remained.

### Waypoint visualization and graph-integrity correction

The first visualization test used the full `rosplane_gcs` launch. It produced
the correct waypoint markers but duplicated `/rviz` and TF publishers already
owned by the simulator. Do not repeat that launch combination.

The first graph-safe `p_start_waypoint_viz` revision was tested with the
simulator and produced:

- existing simulator node `/rviz`
- unique visualization node `/rosplane_waypoint_rviz`
- unique marker node `/rosplane_waypoint_publisher`
- only the simulator's two static-transform publishers
- marker topics `/rviz/waypoint`, `/rviz/mesh`, `/rviz/mesh_path`, and their
  array variants
- no duplicate node names

This still opened two GUI windows. Inspection on 2026-07-22 confirmed that the
simulator's `standalone_sim.rviz` and ROSplane GCS RViz configurations both
subscribe to `/rviz/waypoint`, `/rviz/mesh`, and `/rviz/mesh_path`. The helper
was therefore corrected again to run only `/rosplane_waypoint_publisher` and
reuse the simulator's existing `/rviz` window.

The corrected single-window helper passes `zsh -n` and was verified against the
live `veloxity-plane` session. The graph contains exactly one `/rviz` and one
`/rosplane_waypoint_publisher`, the marker stream is active, and the process
audit found one instance of every expected simulator and ROSplane component.
ROSflight remained armed and offboard with no failsafe and zero errors. During
recovery of the persistent Screen session, its inherited Xauthority cookie had
become stale after Xwayland restarted; exporting the current `XAUTHORITY` in
that old window allowed the single RViz process to reconnect. A fresh Screen
session will inherit the current cookie normally.

The user's live visual check then exposed aircraft shaking. Topic-level endpoint
inspection found the cause despite the clean process count: both
`/standalone_viz_transcriber` and `/rosplane_waypoint_publisher` published
`/rviz/mesh`, `/rviz/mesh_path`, and `/tf`. The two nodes used different state
sources and competed to drive the same RViz aircraft. `p_start_waypoint_viz`
now remaps the ROSplane publisher's mesh, path, and TF outputs under
`/rosplane_waypoint_viz/*`, leaving its `/rviz/waypoint` output unchanged.
After restarting that publisher, `/rviz/mesh`, `/rviz/mesh_path`, `/tf`, and
`/rviz/waypoint` each had exactly one publisher. The isolated ROSplane mesh,
path, and TF topics each had no subscribers, while the simulator RViz remained
connected to the single authoritative simulator visualization streams.

### Documented hourglass mission check

The local upstream `rosplane/missions/fixedwing_mission.yaml` is the ROSplane
2.0 documentation example and already contains the four-point hourglass:
`(400,-250)`, `(-400,-50)`, `(-400,-250)`, `(400,-50)`, all at down `-70 m`
and `17 m/s`. The earlier display showed only three points because
`path_planner` defaults `num_waypoints_to_publish_at_start` to 3. The fourth
point remained loaded but unpublished until `p_publish_next_waypoint` was
called.

For a clean live test, the example mission was reloaded and the fourth waypoint
was immediately published. The aircraft tracked both diagonal hourglass legs at
approximately `17 m/s`, converged near `70 m` altitude, transitioned through
the left corner, and entered the expected `50 m`-radius fillet at the right
corner. ROSflight remained armed, offboard, and error-free.

### Altitude datum diagnosis

The user observed that the truth/path trace flew above the `-70 m` waypoint
plane. Live comparison confirmed a real offset: the ROSplane estimate was about
`p_d=-73.5 m` while simulator truth was about `z=-81.2 m`. This was not a
missed firmware calibration. The startup log shows successful firmware barometer
ground-pressure calibration, followed by the intended IMU calibration and
parameter write.

ROSplane's estimator independently calibrates its initial static pressure from
100 samples after the first arm. The remaining scale error comes from the
installed `estimator.yaml` setting `rho: 1.225`, which overrides the estimator's
air-density calculation based on the simulated site's approximately `1387 m`
origin altitude. The barometer simulator uses the altitude-dependent standard
atmosphere, and the ROSplane GNSS EKF update does not include GNSS altitude, so
vertical position is primarily constrained by the mismatched pressure model.
The correction is implemented in fixed-wing `commands.zsh`:
`p_start_estimator` passes
`ROSPLANE_ESTIMATOR_RHO` after the installed parameter file, with a default of
`-1.0`. This disables the fixed density override and allows the estimator's
calculated local air density to be used. Changing the ROS parameter after
initialization is insufficient because the internal density is selected during
GNSS initialization; the estimator must be restarted while safely
disarmed/under override.

The stack was then shut down safely (override enabled, disarmed, status
confirmed), force-cleaned, and restarted from scratch with
`ROSPLANE_ESTIMATOR_RHO=-1.0`. Firmware ground-pressure calibration and IMU
calibration both succeeded. The estimator was started before arming, and after
arming the aircraft remained stationary under override for five seconds so its
100-sample pressure baseline completed before autonomy was released. With the
aircraft settled near the commanded altitude, ROSplane estimated `72.75 m`
while simulator truth was `72.11 m`, reducing the previous approximately `8 m`
offset to about `0.6 m` (including sample-time separation and sensor noise).
The final graph had one publisher on each control and displayed visualization
topic, two expected static transforms, one RViz, and one isolated waypoint
publisher. ROSflight remained armed, offboard, failsafe-free, and error-free.

### Veloxity FFI realtime scheduler regression

The later ROSplane spiral was traced below ROSplane to the simulation-only
Veloxity FFI adapter. A bag showed `/imu/data` at the expected rate but no
`/baro`, `/gnss`, or `/airspeed`, leaving `/estimated_state` at zero and causing
the controller to drive a meaningless circular trajectory. Running the
unchanged `v_*` multirotor workflow reproduced the same missing firmware sensor
telemetry, proving this was shared simulation integration behavior rather than
a fixed-wing tuning problem.

The FFI worker was already an independent thread whose outer loop continuously
calls `realtime_scheduler_step()` and busy-spins for `Idle`; `/sil_board/run`
only waits (up to 5 ms) for the most recently submitted IMU generation and
copies PWM output. It does not execute or pace the firmware loop. The mismatch
with Pixracer Pro was the world/service configuration inside that thread:
Pixracer selects `TelemetryRates::bounded_high_rate_transport()` and services
two telemetry streams with `RealtimeServicePolicy::continuous_polling(2)`,
while the FFI adapter retained default upstream telemetry rates and used one
stream with a forced 1-us service spacing. Zero-valued upstream stream rates
mean "eligible every service pass" rather than disabled, so the single-stream
budget repeatedly favored the earliest always-due streams and starved later
barometer/GNSS/airspeed streams.

Only `sim/firmware/src/ffi.rs` was changed to apply the same bounded telemetry
rates and continuous two-stream service policy as Pixracer Pro. No shared core,
hardware board, C++ shim, ROSflight/ROScopter/ROSplane source, or `v_*` command
was changed for this correction.

After `pkill -f -9` cleanup and a fresh unchanged `v_*` startup, the graph had
one simulator, one Veloxity shim, one RViz, and one visualization transcriber.
Before calibration the firmware produced stable rates of 400 Hz IMU, 25 Hz
barometer, and 10 Hz GNSS. All 111 firmware parameters verified; IMU and
barometer calibration and persistent write succeeded. The unchanged hover
mission then flew armed/offboard with no failsafe or firmware errors. At the
settled sample, truth was approximately NED `(-20.27, -0.32, -5.58)` with
near-zero velocity and the estimate was approximately
`(-20.26, -0.10, -5.54)`, confirming the complete sensor-estimator-controller
path rather than telemetry presence alone.

### ROSplane validation after FFI scheduler correction

A ROSplane test was then started from a fully clean host state: the active quad
Screen was closed, all remaining ROS processes were force-killed, process and
Screen audits showed no survivors, and a new `veloxity-plane` session was
created. `p_start_sim` was used without an override, so this exercised its
default Veloxity backend. The graph contained `/veloxity_sil_board` and no C SIL
firmware process, with exactly one simulator and visualization stack.

Before firmware or ROSplane initialization, the restored firmware telemetry
rates were measured at 400 Hz IMU, 25 Hz barometer, 10 Hz GNSS, and 50 Hz
airspeed. The 16 fixed-wing firmware parameters verified, IMU calibration and
persistent write succeeded, and the explicit barometer calibration was run
after the simulator was fully started. The estimator was started with the
dynamic-density correction, the complete four-point documented hourglass was
loaded, and the aircraft was held armed under RC override for six seconds
before autonomy was released.

The evidence bag at `/tmp/rosplane_veloxity_scheduler_fix_20260722` covers
44.66 seconds and 65,880 messages. It contains 17,601 IMU, 1,101 barometer, 440
GNSS, 2,201 airspeed, 17,379 estimated-state, 17,866 truth-state, and 440 status
messages. During that interval the plane traveled 561 m through both line and
fillet-orbit path phases at 17.17 m/s median ground speed and 17.11 m/s median
airspeed. Estimate-to-truth 3D position error was 0.83 m median, 1.06 m p95,
and 1.21 m maximum. Active-path lateral error was 0.88 m median and 8.79 m p95;
the approximately 10 m maximum occurred during path transitions rather than an
expanding spiral. Every recorded status was armed, none was in failsafe, and
all reported zero errors. A later live sample confirmed the mission continued
normally after recording stopped. The user requested that this Veloxity
ROSplane run remain running.

### Four-loop C versus Veloxity comparison

On 2026-07-22, matched C and Veloxity trials were recorded under
`~/bagged_plane_data_2026-07-22`. Each trial began from a forced-clean process
state, used the GUI and isolated waypoint markers, verified the same 16
fixed-wing parameters, calibrated IMU and barometer after simulator startup,
wrote parameters, started every component with the documented `p_*` helpers,
held armed under override for six seconds, and flew for 400 seconds before
recording alignment. Each all-topic bag began at the canonical hourglass line
from `(400,-250,-70)` toward `(-400,-50,-70)` and ended after four complete
returns to that boundary.

The C bag contains 4,293,424 messages over 458.94 seconds; the Veloxity bag
contains 3,527,562 messages over 459.09 seconds. Both are approximately 45.1
GiB. Mean loop times were 114.727 seconds for C and 114.758 seconds for
Veloxity. Every status sample in both bags was armed, with no failsafe and zero
firmware errors. Mean lateral path error was 1.752 m for C versus 1.786 m for
Veloxity, showing close broad path behavior.

The comparison is not noise-only. C's mean signed vertical estimate-minus-truth
error was `-1.733 m`; Veloxity's was `+5.488 m`, producing a 7.220 m physical
height split even though both estimators reported essentially zero mean error
from the commanded 70 m altitude. Both firmware paths forwarded simulated
pressure with no meaningful offset, so the difference is not a sea-level
barometer-calibration mismatch. The absolute firmware `Barometer.altitude`
datum differs from the simulator's relative altitude by about 1500 m in both
runs, and ROSplane ignores that field in favor of pressure relative to its own
100-sample armed baseline.

The systematic difference is telemetry cadence. C published barometer at 100
Hz, airspeed at 100 Hz, and magnetometer at 50 Hz. Veloxity's Pixracer-style
bounded transport published them at 25 Hz, 50 Hz, and 25 Hz respectively.
ROSplane expects barometer within 15 ms and magnetometer within 25 ms. The C
bag had eight estimator warnings, all short IMU delays; the Veloxity bag had
5,132 estimator warnings, including 3,307 stale-baro and 1,824 stale-mag
warnings. The four-times-lower pressure update cadence is the strongest
explanation for the vertical EKF bias.

Veloxity also showed larger motion tails: truth angular-acceleration RMS was
1.220 versus C's 0.947, and p99 was 5.330 versus 1.405. Although external
ROSplane commands were similarly smooth, C's active PWM channels typically
changed every 10--13 ms while Veloxity's typically changed every approximately
50 ms. A real 30 ms neutral-output pulse was captured near Veloxity bag
shutdown while status remained armed/offboard and commands continued; host
scheduling pressure during recorder shutdown may contribute, but the pulse was
present in firmware `/output_raw` as well as simulator PWM.

The complete protocol, averages, raw metrics, warning/outlier analysis,
initialization logs, and reproducibility scripts are stored with the two bags.
The final Veloxity trial was intentionally left running, armed, offboard,
failsafe-free, and error-free at the user's request.

### Telemetry-warning and actuator-cadence diagnosis

The follow-up inspection separated the sensor-telemetry problem from the
actuator problem. Veloxity's simulation firmware control loop already runs at
400 Hz, and its simulated PWM driver accepts every control-loop write; the
fixed-wing mixer's nominal 50 Hz hardware PWM setting is recorded but is not
enforced by that driver. ROSplane sends `MODE_PASS_THROUGH` actuator commands,
so Veloxity's internal controller does not run a rate or attitude PID for this
flight. It copies the commanded forces and torques into the inverted-V-tail
mixer.

Only IMU samples are accumulated and averaged at the 400 Hz control deadline.
For magnetometer, barometer, differential pressure, GNSS, range, battery, and
RC, a newer unconsumed sample replaces the older one. Telemetry publishes the
latest processed value at the configured stream rate; those other sensors are
not averaged.

The Pixracer-style bounded profile used for the first comparison explicitly
limited barometer and magnetometer telemetry to 25 Hz. ROSplane's unchanged
estimator expects barometer gaps no larger than 15 ms (about 100 Hz) and
magnetometer gaps no larger than 25 ms (about 50 Hz). Its `check_sensors()`
method warns whenever those thresholds are exceeded, which explains the 3,307
stale-baro and 1,824 stale-mag warnings in the Veloxity bag. Increasing the
publication rate changes what ROSplane receives; it does not increase the
simulator's physical sensor production rate or change firmware-side averaging.

Reconstructing PWM from every recorded `/command`, including elevator reversal,
inverted-V-tail mixing, and integer-microsecond quantization, exposed a separate
loss/hold after ROSplane's controller output. In the C run the predicted and
actual change counts matched closely. In the Veloxity run the aileron predicted
13,507 changes but produced 4,550, the two V-tail outputs predicted 30,163 and
30,250 but produced 6,047 and 6,027, and throttle predicted 15,210 but produced
7,041. Thus the roughly 50--75 ms actuator holds are not merely smooth external
commands, the internal pass-through controller, PWM quantization, or the
nominal 50 Hz hardware PWM configuration. Commands are being collapsed or held
between `/command` and the firmware PWM calculation.

The captured 30 ms neutral-output pulse is compatible with a temporary
offboard-command service gap exceeding the firmware's 100 ms
`OFFBOARD_TIMEOUT`, followed by recovery. Ten-Hz status telemetry can miss such
a short transition, so armed/offboard status immediately before and after does
not rule it out. This remains a hypothesis until command arrival and PWM output
are measured together.

For the next fixed-wing-only test, the launch selects a simulation FFI profile
named `rosplane_c_sil`. It matches the measured C telemetry rates: 400 Hz IMU
and attitude, 100 Hz barometer and differential pressure, 50 Hz magnetometer
and output-raw, 10 Hz GNSS, 800 Hz RC, and 200 Hz battery. The ordinary bounded
profile remains the default when that environment selection is absent, so the
quad workflow is unchanged. No shared core, physical-board implementation,
ROSflight/ROSplane source, or `v_*` file is changed.

### C-rate and command-transport follow-up tests

Two clean GUI-enabled Veloxity trials were run after force-killing old ROS
processes. Both used only the installed `p_*` helpers to start plane
components, verified all 16 fixed-wing parameters, calibrated IMU and
barometer against the running simulation, wrote parameters, loaded all four
hourglass waypoints, armed under override for the pressure baseline, and then
released control to ROSplane. Focused bags and the stress bag are under
`~/bagged_plane_data_2026-07-22/rate_dive_2026-07-22`.

The first trial used `rosplane_c_sil`. Live measurements were 100.00 Hz
barometer, 50.00 Hz magnetometer, 100.00 Hz airspeed, 400 Hz IMU and attitude,
800 Hz RC, and 200 Hz battery. Its 89.9-second focused bag contained no stale
barometer or magnetometer warnings; the only estimator warning was the
unrelated text `hold`. This confirms that the thousands of prior warnings were
caused directly by the bounded telemetry rates rather than barometer
calibration or estimator initialization. The short segment's mean signed
vertical estimate-minus-truth error was `+1.119 m`, but this transient-rich
sample is not a replacement for the previous 400-second/four-loop comparison.

Actuator cadence was smooth in the C-rate focused bag. Reconstructed command
changes versus actual PWM changes were 3,284 versus 3,196 on aileron, 6,112
versus 6,032 and 6,128 versus 6,052 on the V-tail outputs, and 5,639 versus
5,574 on throttle. Active outputs had approximately 10 ms median change gaps.

The second trial deliberately selected the old `bounded` profile and enabled
an opt-in FFI receive trace. The focused 90-second bag reproduced 581 stale
barometer and 408 stale magnetometer warnings, but actuator cadence remained
smooth: reconstructed versus actual changes were 3,345 versus 3,277, 5,908
versus 5,842, 5,922 versus 5,850, and 3,030 versus 2,990. A following 60-second
all-topic stress bag wrote 5.9 GiB and still retained approximately 10 ms
median V-tail updates. It added only four brief IMU, four GNSS, and three
differential-pressure delay warnings.

The receive trace covers 381.30 seconds and 35,421 UDP datagrams. Every
datagram contained exactly one MAVLink frame; there was no multi-frame batching
inside a datagram. It captured 31,222 OFFBOARD_CONTROL frames over 312.63
seconds at 99.866 Hz. Median, p95, and p99 arrival gaps were 9.999, 10.031, and
10.058 ms. Only six gaps exceeded 15 ms. One 338.078 ms delivery gap occurred
between the two recordings while host-side analysis and recorder transitions
were occurring; this is long enough to cross the 100 ms `OFFBOARD_TIMEOUT` and
produce a neutral fallback. No greater-than-100-ms gap occurred within either
recorded flight segment.

These tests refine the diagnosis. The bounded profile definitively causes the
ROSplane freshness warnings and must not be used for fixed-wing parity. It does
not by itself cause the old 50--75 ms actuator holds. The current build accepts
nearly every quantized command in both telemetry profiles, even under a short
all-topic recorder load. The traced 338 ms host-side command-delivery pause
shows a mechanism for isolated neutral pulses during recorder transitions, but
the old sustained stepping was not reproducible and likely depended on the
earlier running build/process state rather than the present scheduler and
adapter. The final live session was restored to the C-rate profile and left
running as `veloxity-plane` with the GUI, armed/offboard, no failsafe, and zero
firmware errors.

### Remaining C-rate vertical datum offset

The restored live C-rate session was sampled for one complete 114.9-second
hourglass loop. The persistent signed vertical estimate-minus-truth error was
`+3.902 +/- 0.173 m`. ROSplane's estimated altitude averaged only `-0.172 m`
from the 70 m command, while physical truth averaged `+3.730 m` above it. The
old bounded-rate Veloxity result was `+5.488 m`, so matching C telemetry removed
about 1.59 m but not the complete offset. Historical C firmware measured
`-1.733 m`; the remaining current-Veloxity versus historical-C split is
5.635 m.

This is not a pressure-forwarding error. Over the live loop firmware pressure
minus raw simulator pressure averaged `+0.036 Pa`. Regression of pressure
against truth and the estimator state decomposes the current `+3.902 m` almost
exactly into two effects above that firmware boundary:

- `+2.464 m` from pressure scale: ROSplane calculated `rho=1.06995 kg/m^3`, or
  `rho*g=10.496 Pa/m`, while the simulated pressure observed over the loop had
  a local slope of `10.145 Pa/m`.
- `+1.438 m` from an inferred pressure-zero/datum term: ROSplane's effective
  pressure baseline was about 15.1 Pa below the ground-pressure intercept
  inferred from the loop.

The same decomposition explains why the historical C run looked better even
though it was not unbiased. It had a `+1.852 m` density-scale term and a
`-3.585 m` inferred datum term, which happened to cancel to `-1.733 m`.
Therefore the smaller visible C offset was favorable cancellation, not proof
that C forwarded a more correct barometer value. Most of the 5.635 m cross-run
difference is the estimator pressure-zero term changing between
initializations; the density-model term accounts for the rest. The pressure
zero is inferred from the EKF state and pressure regression because ROSplane
does not expose `init_static_` as a ROS parameter or service.

The loop bag and metrics are saved as
`~/bagged_plane_data_2026-07-22/rate_dive_2026-07-22/veloxity_c_rate_live_loop`
and `c_rate_live_loop_metrics.json`. The live GUI flight was left running.

### Parameterized telemetry deadlines

Veloxity now appends 12 firmware parameters for telemetry publication. Their
defaults match the measured ROSflight C SIL rates: heartbeat/status 1/10 Hz,
IMU/attitude 400/400 Hz, output 50 Hz, differential pressure/barometer 100/100
Hz, magnetometer/range 50/50 Hz, battery 200 Hz, GNSS 10 Hz, and RC 800 Hz.
The parameter surface therefore contains 349 entries; the 337 count above
describes the earlier validation build.

The parameter names are `TEL_HB_HZ`, `TEL_STATUS_HZ`, `TEL_IMU_HZ`,
`TEL_ATT_HZ`, `TEL_OUT_HZ`, `TEL_DIFF_HZ`, `TEL_BARO_HZ`, `TEL_MAG_HZ`,
`TEL_RANGE_HZ`, `TEL_BATT_HZ`, `TEL_GNSS_HZ`, and `TEL_RC_HZ`. A value of -1
disables the stream, 0 makes it eligible on every service opportunity, and
1--2000 requests a fixed rate in hertz. Live changes restart only that stream's
deadline and do not alter sensor acquisition or the control-loop rate.

For C/Rust altitude comparisons, do not tune firmware `BARO_BIAS` or
`GROUND_LEVEL` to remove the ROSplane offset. Use identical stationary startup
and estimator-calibration timing, record the preflight pressure baseline, and
hold ROSplane density constant between paired runs. Report both raw altitude
error and pressure-baseline-aligned error. Use `rho=-1` for upstream-fidelity
runs; use a simulator-derived fixed density for controlled estimator runs.

## Current manual startup sequence

```zsh
p_start_screen
screen -r veloxity-plane
```

The interactive-shell startup file loads both the `v_*` and `p_*` helper
definitions. `p_start_screen` only creates idle, named interactive shells; it
does not inject text or run any component command in those windows. Start each
component manually with the corresponding `p_start_*` command below.

Then run one command in each like-named Screen window, in this order:

1. In `firmware`, run `p_start_sim` (Veloxity default). To run C, set
   `ROSPLANE_FIRMWARE=c` before sourcing fixed-wing `setup.zsh` and creating
   the Screen session. On Pixracer Pro hardware, run `p_start_uart` or
   `p_start_usb` instead, never both.
2. In a separate shell, run `p_start_sim_rviz`.
3. Run `p_load_firmware_params` or `p_load_firmware_snapshot FILE` after
   `rosflight_io` reports a connection/parameters, then calibrate explicitly.
4. `p_start_estimator`
5. `p_start_truth`
6. `p_start_path_planner`
7. `p_start_path_manager`
8. `p_start_path_follower`
9. `p_start_controller`

Load a readable mission from the `commands` window with
`p_load_mission /absolute/path/to/mission.yaml`. Run `p_help` for the complete
command list. On a fresh no-joystick simulator, run `p_toggle_sim_arm` first,
then `p_toggle_sim_override` to release control to ROSplane. Run
`p_start_waypoint_viz` in the `gcs` window for waypoint markers; `p_start_gcs`
is a compatibility alias to that same graph-safe helper.

## Open questions

- After basic startup is verified, which aircraft should be the long-term
  default: the simulator's current `anaconda` dynamics or another installed
  fixed-wing parameter set?

## Next steps

1. Repeat the saved 400-second/four-loop Veloxity protocol with the now-default
   fixed-wing C-SIL telemetry profile to determine the long-duration vertical
   bias after the short validation eliminated freshness warnings.
2. Decide whether to keep `anaconda` as the long-term dynamics/autopilot default
   after observing a real flight attempt.
3. If needed, add a user-selected default mission path. Do not change firmware
   or ROSflight/ROSplane.
4. Record each user-observed result, decision, failed attempt, and next action in
   this file before changing direction.
