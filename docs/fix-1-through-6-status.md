# Fix 1-6 Status

This file tracks the six remaining parity test areas named during the
ROSflight C parity review. Sim/unit coverage is the current gate. Hardware
validation remains a separate gate after sim coverage is clean.

## Fix 1: Numeric C-Sample Comparisons

Status: covered by unit fixtures, with room for external C trace expansion.

- Sensor compensation fixtures now cover:
  - IMU orientation-before-bias correction.
  - IMU gyro bias, accel bias, and accel temperature compensation.
  - Magnetometer orientation, hard-iron subtraction, and row-wise soft-iron
    matrix multiplication.
  - Barometer pressure-to-altitude correction and calibration timing.
  - Pitot bias correction and signed airspeed formula.
  - Battery low-pass initialization and second-sample behavior.
- Controller/PID fixtures now cover:
  - PID derivative filtering, integration, saturation, and disabled integrator.
  - Angle-mode body-rate derivative feedback.
  - Rate-mode output with equilibrium torque offsets.

Follow-up: generate fixture values from an instrumented C firmware run and keep
them beside the Rust tests if byte-for-byte trace provenance is required.

## Fix 2: Fixed-Wing Paths

Status: unit fixtures covered; fixed-wing SIL smoke path now demonstrated
against the same ROSplane sequence as the C baseline. Dedicated fixed-wing
vehicle pipeline coverage is still open.

- Fixed-wing RC input maps primary control axes to passthrough.
- Fixed-wing failsafe accepts passthrough throttle values outside the multirotor
  0-1 throttle range.
- The existing attitude estimator honors `PARAM_FIXED_WING` by not failing
  health only because accelerometer correction timed out. This is a fixed-wing
  flag branch inside the current attitude estimator, not a dedicated fixed-wing
  estimator implementation.
- Canned fixed-wing mixer output types, default PWM rates, servo/throttle
  mapping, and reversal params are tested through the current mixer module.
- Primary/secondary row selection is tested against ROSflight C override-mask
  behavior.
- Fixed-wing ROSplane SIL baseline:
  - The upstream tutorial path uses VimFly, and with VimFly active `rc.py` does
    not expose `/toggle_arm` or `/toggle_override`.
  - A no-VimFly service path exists through `rc.py` when VimFly is disabled,
    but it is not the same scenario as the upstream ROSplane tutorial. The
    tutorial's validated user flow is the VimFly/manual path.
  - With the same override-release sequence, Voloxide reaches `armed=true`,
    `rc_override=0`, `offboard=true`, finite `/estimated_state`, finite
    `/command`, and publishing `/airspeed` and `/sim/pwm_output`.
- Fixed-wing parameter-load and mixer-reflection anchor:
  - ROSflight IO handles YAML parameter files by looping over each matching
    entry in `ParamManager::load_from_file()`, enqueuing each outgoing
    `PARAM_SET` in `param_set_queue_`, then sending one queued MAVLink message
    per `param_set_timer_callback()` tick. This is a ROS-side `std::deque`, not
    a large firmware-side bulk queue.
  - The C firmware handles each received `PARAM_SET` immediately through
    `Mavlink::handle_msg_param_set()` and the parameter callbacks. It does not
    need to absorb an arbitrary YAML burst at the firmware boundary because
    ROSflight IO serializes that burst.
  - When `PRIMARY_MIXER` or `SECONDARY_MIXER` changes, C
    `Mixer::param_change_callback()` calls `init_mixing()`. For canned
    fixed-wing mixers, `init_mixing()` loads the raw fixed-wing matrix without
    inversion and `save_primary_mixer_params()` writes the reflected
    `PRI_MIXER_OUT_*`, `PRI_MIXER_PWM_*`, and `PRI_MIXER_*_*` parameters that
    ROSflight IO and the sim dynamics read back.
  - Voloxide previously collapsed multiple inbound `PARAM_SET` messages into a
    single slot and then a 4-entry event queue. The fixed-wing init YAML burst is
    larger than that, so some settings could be dropped before the mixer
    reflection was refreshed.
  - Current Voloxide firmware preserves inbound `PARAM_SET` messages FIFO at the
    comm ingress, moves them into the ECS event queue only while that downstream
    queue has room, refreshes reflected mixer params when mixer choice params are
    set, and emits `PARAM_VALUE` responses for the reflected mixer params so the
    ROSflight IO cache updates.
  - Fixed-wing visual smoke on Voloxide reached `armed=true`, `rc_override=0`,
    `offboard=true`, and published waypoint markers. A later focused run showed
    Voloxide accepting ROSplane throttle (`/command.u[0]=1.0`) and publishing
    motor output on fixed-wing channel 4 (`/sim/pwm_output.values[4]=2000`).
  - The C ordering is important: `Params::set_param_int(PRIMARY_MIXER)` calls the
    mixer change callback first; the callback writes `PRI_MIXER_*` params and
    those internal writes emit their own `PARAM_VALUE` messages before the direct
    `PRIMARY_MIXER` acknowledgement is emitted. Voloxide now mirrors that
    response order for mixer-choice writes so ROSflight IO and the fixed-wing
    dynamics node do not observe the mixer choice before the reflected mixer
    cache has been refreshed.
  - The same reconciliation must also happen on firmware startup. A live
    fixed-wing smoke found a persisted `voloxide_sim.params` state with
    `PRIMARY_MIXER=10` and `FIXED_WING=1`, but stale quad `PRI_MIXER_*`
    reflection. ROSflight IO then skipped the YAML `PRIMARY_MIXER=10` write
    because its cached value was already 10, so no mixer-change callback fired.
    Voloxide now refreshes reflected mixer params during `World::init`, matching
    C `Mixer::init()` boot behavior.
  - The comm-ingress FIFO capacity covers the known ROSplane tutorial init burst
    and keeps the firmware bounded. Correctness should come from serialized
    drain/backpressure behavior, not from describing any fixed queue size as
    arbitrary bulk-load parity.
  - A later fixed-wing visual run exposed a separate launch/runtime issue. The
    reused `rmw_zenohd` router kept stale ROScopter graph state visible during a
    ROSplane fixed-wing run (`/waypoints`, `/trajectory_command`,
    `/sim/roscopter/state`), which made topic inspection ambiguous. The
    fixed-wing demo script now restarts Zenoh by default so each visual run gets
    a clean graph; set `RESTART_ZENOH=false` only when deliberately sharing a
    router.
  - The same run also showed a sporadic `sil_board/run service client timed
    out` warning from `rosflight_sil_manager`. ROSflight's default service
    result timeout is 10 ms; Voloxide can briefly exceed that during heavier
    MAVLink/param/telemetry cycles. The Voloxide standalone launch files now set
    `service_result_timeout_ms=100` for the SIL manager while leaving the
    upstream ROSflight sim code unchanged.
  - Fixed-wing forces can still be zero if `fixedwing_forces_and_moments`
    performs its first firmware parameter cache load before ROSflight IO has
    completed the fixed-wing YAML load. In that state it observes zeroed mixer
    reflection, treats channel 4 as non-motor, and publishes zero forces even
    while Voloxide publishes channel 4 at 2000 us. Publishing
    `/status/params_changed` after `/all_params_received` returns success forces
    the dynamics node to reload `PRI_MIXER_OUT_*` and `PRI_MIXER_*_*`; the
    captured Voloxide run then reported `PRI_MIXER_OUT_4=2`,
    `PRI_MIXER_0_4=1`, and nonzero `/sim/forces_and_moments`. The ROSplane demo
    script now performs that refresh immediately after
    `fixedwing_init_firmware.launch.py`.
  - The apparent visual "glitching" was reproduced against the upstream C
    firmware too, so it is not a Voloxide mixer regression. ROSplane's estimator
    starts estimating only after the firmware first reports `armed=true`; in the
    local ROSplane 2.0 tree, the low-pass airspeed member can enter that first
    armed update with stale/uninitialized state. When that value is huge, the
    controller drives throttle to zero and the estimator repeatedly resets.
    The fixed-wing demo script now applies launch-time estimator parameters
    before arming: `airspeed_cutoff_freq=1000.0` so the first differential
    pressure sample overwrites the stale airspeed quickly, plus fixed magnetic
    `inclination=67.0` and `declination=11.0` so heading initialization does
    not depend on runtime WMM lookup timing. With those parameters, the C
    baseline produced finite `va` near 17 m/s, throttle near 0.86, nonzero
    forces, and moving truth state; Voloxide produced the same healthy shape
    with finite `va` near 17 m/s, throttle near 0.74, nonzero forces, and moving
    truth state.
  - A visual RViz run then exposed two launch hygiene issues not present in the
    non-GUI smoke. First, the standalone RViz launch already publishes the
    `world -> NED` and `aircraft_body -> stl_frame` static transforms, so the
    waypoint-visualization block no longer starts duplicate static transform
    publishers. The ROSplane GCS `rviz_waypoint_publisher` also publishes an
    estimated-state aircraft mesh and `NED -> aircraft_body` transform, which
    competes with the standalone sim's truth-state aircraft visualization and
    can make RViz appear to resize or jitter. The demo script now uses a
    Voloxide waypoint-only marker publisher that publishes `/rviz/waypoint`
    markers without publishing aircraft TF or mesh, and sends `DELETEALL` on
    startup so stale RViz waypoint/path markers from previous runs are cleared.
    A later live GUI run confirmed the remaining RViz camera/aircraft jitter was
    from two real stale `standalone_viz_transcriber` processes left alive from
    older runs. They continued publishing old `/tf` transforms while the current
    sim published its live aircraft transform. After killing only those stale
    PIDs, `/tf` reported exactly one publisher. The demo script now performs a
    preflight stale-process sweep and explicitly matches
    `standalone_viz_transcriber` before launching a new fixed-wing visual run.
    Second, the script now calls `/dynamics/set_sim_state` before arming,
    matching the ROSplane tutorial's reset guidance and ensuring the visual run
    starts from a finite standalone dynamics state. Third, ROSplane's path
    planner publishes only three waypoints by default; the script now sets
    `num_waypoints_to_publish_at_start=100` before loading the mission so all
    waypoints in the default fixed-wing mission are plotted and available at
    startup.
  - The quad waypoint script showed two launch behaviors worth carrying back
    into fixed-wing and vice versa. Quad keeps the vehicle under RC override
    through estimator/calibration startup and only releases override after the
    mission/autonomy stack is ready; fixed-wing keeps the same override handoff
    discipline and follows the ROSplane tutorial assumption that the fixed-wing
    vehicle starts on the ground. The fixed-wing demo now delays RViz startup
    until after standalone dynamics has received a finite ground state, then
    re-seeds the same finite ground state before arming. This prevents RViz from
    ingesting early NaN aircraft transforms without inventing an airborne
    initial condition. The quad demo now uses the same preflight stale-process
    sweep and optional Zenoh restart pattern as the fixed-wing demo so RViz/ROS
    graph state is clean between vehicle types.
  - A follow-up C-firmware comparison used the same fixed-wing script with
    `FIRMWARE=c`, `USE_VIMFLY=false`, finite ground state, delayed RViz,
    mission load, `/toggle_arm`, and `/toggle_override`. The visual graph was
    clean (`/tf` had exactly one `standalone_viz_transcriber` publisher), but
    this service-only path did not reproduce the tutorial flight. ROSplane kept
    publishing zero `/command.u`, `/sim/RC` only reached the intended armed and
    override-disabled switch values after manually repeating both service
    calls, and `/sim/pwm_output`/`/output_raw` did not produce active samples.
    This anchors the current difference to launch/control sequencing versus the
    docs' VimFly/manual path, rather than proving a new Voloxide-only fixed-wing
    mixer regression. The fixed-wing script now exposes `FIRMWARE=voloxide|c`
    so future C/Voloxide comparisons use the same launch hygiene.
  - A later minimal fixed-wing SIL-loop validation isolated the earlier missing
    `/sim/pwm_output` symptom to startup ordering, not firmware compatibility.
    With `rmw_zenohd` already running before launch, the C baseline published
    `/sim/sensors/imu/data` at 400 Hz, `/sim/pwm_output` at 400 Hz, and
    `/output_raw` at about 47.5 Hz; `/sil_board/run` also returned success from
    a fresh CLI client. Under the same conditions Voloxide published
    `/sim/sensors/imu/data` at 400 Hz, `/sim/pwm_output` at 400 Hz, and
    `/output_raw` at 50 Hz, with ROSflight IO printing the same fixed-wing
    primary and secondary mixer reflection as C. The fixed-wing launch now gives
    `rosflight_sil_manager` a 100 ms service-exists timeout and the demo script
    waits for the Zenoh router before launching ROS nodes, matching the clean
    baseline ordering without changing upstream ROSflight code.
  - The manual VimFly handoff is not deterministic enough for repeatable
    acceptance runs: the `r` key toggles one RC switch, but firmware
    `rc_override` is a reason bitmask and can legitimately reassert if a stick
    or throttle channel deviates. The ROSplane tutorial wrapper now defaults to
    a Voloxide-side deterministic RC publisher that keeps the channels centered,
    arms under override, waits for a live ROSplane `/command`, releases the
    override switch, and waits for stable `armed=true` with `rc_override=0`.
    Because the validation wrapper uses a finite in-flight fixed-wing state, it
    also ramps manual throttle from 1000 us to 1600 us while override is still
    active before releasing to computer control. That preserves the same C
    firmware RC switch contract while avoiding an artificial zero-throttle to
    ROSplane-throttle step during visual acceptance runs.
  - A later visual run exposed two non-firmware diagnostic hazards. First, if
    `rmw_zenohd` exits or is not reachable when nodes initialize, rmw_zenoh can
    leave the graph partitioned after one router-check attempt. The fixed-wing
    wrapper now exports `ZENOH_ROUTER_CHECK_ATTEMPTS=20`, waits 5 seconds after
    starting `rmw_zenohd`, and aborts before launching ROS nodes if the router
    process is not alive. Second, ad hoc `ros2 topic` diagnostics must run with
    the same middleware as the demo:
    `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Without that export, topic
    listing and endpoint inspection can appear partially useful while
    `ros2 topic echo` or `ros2 topic hz` misses the live
    data plane, which falsely looks like a firmware or sim failure.
  - With the smoother handoff and corrected Zenoh startup, the active visual
    Voloxide fixed-wing run reported `/sim/pwm_output` at about 400 Hz,
    `/status` as `armed=true`, `failsafe=false`, `rc_override=0`,
    `offboard=true`, `error_code=0`, and finite moving `/sim/truth_state`.
  - A C/Voloxide SIL-board comparison found one remaining board-boundary
    difference relevant to tutorial takeoff. The upstream C SIL board
    initializes RC channels to centered sticks, low throttle, low override, and
    low arm in `SILBoard::pwm_init()`. Its `rc_read()` returns those initialized
    values whenever a `/sim/RC` publisher exists, even before the first callback
    updates them; if no publisher exists, it still fills a low-throttle/centered
    struct before returning "no RC." The Voloxide shim previously exposed no RC
    snapshot until the first `/sim/RC` message arrived. The shim now initializes
    the same default RC channel shape and exposes it when a `/sim/RC` publisher
    is present, while still reporting no RC when no publisher exists. This
    mirrors the C SIL boundary without changing ROSflight or ROSplane.
  - The same comparison clarified why the documented C path can appear to work
    from takeoff while a truth-state adapter fails at ground start. C uses the
    stock ROSplane estimator/control topology from `rosplane_sim sim.launch.py`;
    the local Voloxide truth-state adapter made `sim_state_transcriber` output
    control-critical. At zero airspeed, that transcriber computes sideslip from
    `va_y / state.va`, so a ground state can publish NaN truth state. A true
    C-like fixed-wing takeoff validation should therefore use the stock ROSplane
    estimator path plus C-like RC startup behavior, not the truth-state adapter.
  - After adding the C-like RC startup behavior, a headless Voloxide run with
    `USE_TRUTH_STATE_AUTONOMY=false`, zero-speed ground seed, stock
    `rosplane_sim sim.launch.py`, and deterministic RC handoff still failed at
    the first armed estimator update. ROSflight IO reported RC recovery,
    `PASS_THROUGH`, and `Autopilot ARMED`, then ROSplane immediately emitted
    extreme roll/pitch/yaw warnings, GPS limit warnings, and repeated
    `Estimator reinitialized due to non-finite state` messages before override
    release completed. This means the RC-default mismatch was real and fixed,
    but it is not the only difference between the documented C/VimFly takeoff
    path and the current Voloxide automated ground-start path.
  - The same visual stock-estimator ground-start experiment was repeated with
    `FIRMWARE=c` through the same wrapper and reproduced the estimator
    non-finite reset cascade, RViz NaN TF warnings, and controller NaN output
    after arming. That makes the forced zero-speed visual ground-start path a
    ROSplane fixed-wing startup-condition problem rather than a Voloxide
    firmware parity bug. The script now defaults stock ROSplane estimator runs
    to start ROSplane from the finite fixed-wing release state
    (`17 m/s`, `down=-70 m` by default), while preserving explicit
    `ROSPLANE_START_AIRSPEED` and `ROSPLANE_START_DOWN_POSITION` overrides for
    experiments.
  - A visual run showed that keeping a separate ground visual seed and then
    reseeding the same finite release state during handoff still creates
    visible discontinuities. Starting the firmware calibration from the finite
    flight state also trips movement/baro calibration warnings. The wrapper now
    seeds a ground state for firmware calibration while RViz is still closed,
    then seeds the finite ROSplane startup state before opening RViz and
    starting ROSplane. Stock-estimator handoff no longer reseeds release state
    by default; release-state reseeding is reserved for truth-state adapter runs
    or explicit overrides.
  - The deterministic fixed-wing RC helper also no longer waits through a
    default manual throttle warmup. It now uses override only to arm and confirm
    live ROSplane commands, then releases immediately by default. This is closer
    to the ROSflight 2.0 tutorial handoff model than trying to manually fly the
    fixed-wing plant before autonomy takes over.
  - The handoff helper keeps throttle low until `/status.armed` is true. This
    preserves ROSflight's arming guard (`Cannot arm with RC throttle high`) while
    still allowing an optional post-arm manual throttle warmup for experiments.
  - Visual testing showed the deterministic helper is still the wrong default
    for fixed-wing ROSplane validation: it can produce a technically armed and
    offboard firmware state while the aircraft visibly rolls/falls because a
    fixed-wing aircraft needs a real takeoff/trim transition. The visual
    tutorial wrapper now defaults to VimFly/manual RC and the ROSplane
    controller/path stack fed from the local truth-state adapter. The
    deterministic helper was removed after this validation pass so the
    maintained script surface only exposes the manual handoff path.
  - The VimFly/manual path also failed when the stock ROSplane estimator was
    already running at a zero-speed ground state: after `/status.armed=true`
    and before RC override release, ROSplane emitted extreme attitude/GPS
    warnings and repeated non-finite estimator resets. Assisted seeding to the
    nominal airborne waypoint state after that point did not recover the
    estimator. This means the remaining fixed-wing visual failure is not just a
    deterministic handoff artifact; the exact C/VimFly timing must be compared
    next, especially whether the C tutorial avoids having ROSplane's estimator
    armed and integrating while the fixed-wing sim is still stationary on the
    ground.
  - The tutorial wrapper has therefore been simplified around that finding:
    launch Voloxide fixed-wing SIL with VimFly and RViz, calibrate on the
    ground, pause for a manual VimFly takeoff, then start ROSplane's controller
    and path nodes from `/sim/rosplane/state`, load the waypoint mission, and
    prompt for RC override release. This avoids running the stock ROSplane
    estimator while the fixed-wing aircraft is stationary on the ground and
    returns the visual demo to the truth-state path that previously followed
    waypoints.
  - The local ROSplane truth-state path is also not valid at zero airspeed:
    `sim_state_transcriber` computes sideslip as `asin(va_y / state.va)`, so a
    ground state with `state.va == 0` publishes NaN truth state. The visual
    validation wrapper now separates visual startup from ROSplane handoff. It
    seeds a zero-speed ground state before ROSplane exists so RViz and firmware
    calibration start visually on the ground, then seeds a small nonzero
    near-ground state before ROSplane subscribes. The deterministic RC handoff
    arms under override at that near-ground state, waits for live ROSplane
    `/command`, seeds the finite release state
    (`RC_HANDOFF_RELEASE_AIRSPEED=17.0`,
    `RC_HANDOFF_RELEASE_DOWN_POSITION=-70.0` by default), and only then releases
    RC override. It seeds both `/dynamics/set_sim_state` and one matching
    `/sim/truth_state` sample because the standalone dynamics node only
    publishes truth after it receives forces, while the fixed-wing forces node
    computes its first forces from cached truth. The direct finite truth seed
    breaks that startup dependency without changing upstream ROSflight or
    ROSplane code. The zero-speed ROSplane truth path remains a ROSplane sim
    limitation to investigate separately, not a Voloxide firmware parity failure.

Follow-up: add or wire a dedicated fixed-wing vehicle pipeline and run a
fixed-wing SIL mission if the ROSflight 2.0 sim environment has a maintained
fixed-wing scenario available. Quadrotor controller/mixer/estimator tests do not
prove full fixed-wing parity. The current fixed-wing SIL evidence is a smoke
test of the ROSflight firmware boundary and ROSplane integration sequence, not a
full fixed-wing flight envelope validation.

## Fix 3: Telemetry Stream Rates

Status: unit cadence covered; C and Voloxide SIL snapshots captured for the
core default streams.

- Unit fixture covers the ROSflight C defaults:
  - heartbeat at 1 Hz,
  - status at 10 Hz,
  - IMU and attitude every IMU update,
  - output raw every 8 IMU updates,
  - non-IMU telemetry independent of IMU control updates.

SIL snapshots:

- C ROSflight SIL baseline:
  - `/status`: about 10.000 Hz.
  - `/output_raw`: about 50.0 Hz.
  - Correct IMU topic is `/imu/data`; the first C snapshot used the stale
    `/imu` topic name and should be rerun if a direct C IMU topic-rate number is
    needed in the report.
- Voloxide SIL with Zenoh and RViz enabled:
  - `/status`: about 9.90 Hz, min/max about 0.099-0.103 s.
  - `/output_raw`: about 50.0 Hz, min/max about 0.019-0.021 s.
  - `/imu/data`: about 400.0 Hz, min/max about 0.001-0.004 s.

Follow-up: rerun the C baseline `/imu/data` measurement under the same Zenoh and
RViz-enabled setup if a matched final table is required.

Version note: the Voloxide runtime publishes `/version` as `Voloxide 1.0`.
ROSflight IO's warning log strips any leading `v`/`V` before comparing versions,
so that log currently prints `oloxide 1.0` even though the raw version topic is
correct.

## Fix 4: Embedded Board Paths In Sim

Status: sim-proxy coverage added for board-owned behavior.

- PWM:
  - enable/disable follows state transitions,
  - disabled writes are suppressed,
  - motor, servo, GPIO, and aux ownership compose correctly,
  - motor/servo/GPIO ranges clamp correctly,
  - mixer default rates propagate to the PWM resource.
- LEDs:
  - RC override drives LED0.
  - Disarmed/error/armed/failsafe LED1 intent is covered by world-level tests.
- Persistent params:
  - read/write board command path round-trips through the board-owned params
    interface while disarmed.
  - command path rejects unsafe board commands while armed.
- Backup memory:
  - valid backup data is cleared once, reported after companion heartbeat, and
    routes rearm intent through the state-machine transition.
- RC parsing:
  - valid frames and lost frames are covered through the RC command state.
- Sensor init/read failure:
  - board sensor error counts are surfaced in status telemetry.

Follow-up: add board-specific integration tests when hardware is attached.

## Fix 5: Hardware Validation

Status: not complete until physical boards are available.

Hardware validation must cover:

- PWM pulse ranges and update rates on Pixracer/Nucleo outputs.
- LED behavior on actual board pins.
- Persistent param read/write/defaults after reboot.
- Backup memory valid/invalid/rearm behavior across reset.
- RC parser behavior with real receiver frames and lost-link transitions.
- Sensor init failure behavior for each attached board sensor.

## Fix 6: Commit Readiness

Status: pending final validation.

Required checks before commit:

- `cargo fmt --check`
- `git diff --check`
- `cargo check -p voloxide_core --lib --message-format=short`
- `cargo test -p voloxide_core --lib`
- `cargo test -p sim --lib`
- embedded target checks for `stm_32`, `pixracerpro`, and `nucleo`

## ROSplane Tutorial Automation Notes

Status: fixed-wing tutorial parity still under investigation.

- Added `scripts/run_voloxide_rosplane_tutorial_demo.zsh` as a tutorial-shaped
  wrapper around the fixed-wing demo. It defaults to:
  - Voloxide firmware endpoint,
  - VimFly enabled,
  - standalone RViz enabled,
  - waypoint-only marker publishing enabled,
  - ROSplane GCS disabled,
  - ground start at zero airspeed and zero down position.
- VimFly remains a manual step because this workstation does not currently have
  `xdotool` or `wmctrl` available for reliable focused-window key injection.
  The wrapper prompts for the tutorial sequence: click VimFly, press `t`, wait,
  press `r`, then press Enter in the terminal.
- Voloxide-backed tutorial run, 2026-05-20:
  - reached the VimFly arming step,
  - reported `Autopilot ARMED`,
  - immediately drove ROSplane estimator warnings with extreme roll/pitch/yaw,
    GPS limit warnings, and repeated non-finite state reinitializations.
  - The captured failure run showed the important Rust-side difference before
    arming: ROSflight IO received the fixed-wing primary mixer correctly, but
    the secondary mixer cache was all zeros. When VimFly returned to computer
    control, Voloxide's C-compatible primary/secondary row selection used the
    secondary rows for the no-override path, so fixed-wing control entered the
    sim with no valid secondary actuator mapping. ROSplane then reported NaN
    roll-loop controller terms and non-finite estimator states immediately
    after override release.
- C-backed comparison run with the same wrapper, 2026-05-20:
  - command used `FIRMWARE=c scripts/run_voloxide_rosplane_tutorial_demo.zsh`,
  - reached the same VimFly arming and RC override release path,
  - reported `Autopilot ARMED`, `Returned to computer control`, and ROSplane
    controller `climb`/`hold` transitions,
  - did not reproduce the Voloxide non-finite estimator blow-up,
  - still showed frequent `imu sensor not received` warnings, `/command` was
    publishing but a sampled message was all zeros, and `/output_raw` was not
    observed during the sample window.

Root cause found so far: the estimator explosion was triggered by a
Rust-firmware secondary-mixer parity bug, not by ROSplane's estimator code
alone. C treats an invalid/unset secondary mixer as "default to primary" during
mixer initialization. Voloxide's reflection path only treated values greater
than or equal to the mixer count as invalid, so the default
`SECONDARY_MIXER=-1` was not refreshed and ROSflight IO saw a zero secondary
matrix. `voloxide_core/src/mixer/matrix.rs` now treats any value outside
`0..NUM_MIXERS` as invalid for reflection and mirrors the primary matrix when
the secondary choice is unset.

Post-fix evidence:

- `cargo test -p voloxide_core
  reflected_secondary_mixer_defaults_to_primary_when_choice_is_unset --lib`
  passed.
- `cargo build -p sim --lib` passed.
- `colcon build --packages-select voloxide_sil_board_shim --symlink-install`
  passed.
- A headless Voloxide fixed-wing run after the fix showed ROSflight IO receiving
  matching primary and secondary fixed-wing matrices, reached
  `Autopilot ARMED` and `Returned to computer control`, and did not reproduce
  the repeated non-finite estimator reset cascade in the monitored window.

Remaining validation: rerun the VimFly/tutorial visual path with the rebuilt
shim to confirm the exact manual override-release scenario now stays finite.
The C run was not treated as a validated waypoint-flight success because output
commands were not yet proven to reach PWM in that run.

Follow-up comparison, 2026-05-20:

- A Voloxide visual tutorial rerun using an externally held `rmw_zenohd` router
  reached the same VimFly sequence, reported `Autopilot ARMED`,
  `Returned to computer control`, ROSplane `climb`, and ROSplane `hold`, and
  did not reproduce the non-finite estimator reset cascade. The secondary mixer
  cache matched the primary fixed-wing matrix.
- In that Voloxide run, `/command` sampled near 90 Hz, but CLI sampling of
  `/sim/sensors/imu/data`, `/sim/pwm_output`, `/sim/forces_and_moments`,
  `/sim/truth_state`, and `/output_raw` timed out even though the ROS graph
  showed the expected publishers/subscribers.
- The same tutorial path was then rerun with `FIRMWARE=c` and the same external
  Zenoh router. C also reached `Autopilot ARMED`, `Returned to computer
  control`, ROSplane `climb`, and ROSplane `hold`. It also showed the same CLI
  sampling symptom: `/command` sampled near 90-100 Hz while
  `/sim/sensors/imu/data`, `/sim/pwm_output`, and `/output_raw` did not sample.

Interpretation: the missing `/sim/pwm_output` and sim-topic samples are not
specific to Voloxide firmware behavior. They reproduce with the upstream C
firmware under the same fixed-wing tutorial/Zenoh/visual launch path, so they
should be investigated as a ROS 2/Zenoh/SIL stepping or launch/runtime issue.

Manual-takeoff wrapper update, 2026-05-20:

- The visual script was changed to pause before ROSplane startup, allowing a
  manual VimFly takeoff under RC override before autonomy is launched.
- A run using that flow with the stock `rosplane_sim sim.launch.py` still drove
  the ROSplane EKF non-finite immediately after ROSplane startup, before RC
  override release. At the same time `/sim/truth_state` was finite and
  plausible, and firmware `/status` reported `armed=true`, `rc_override=3`,
  `offboard=true`, `error_code=0`.
- Because the failure occurred before handoff, the simple visual waypoint demo
  now defaults back to the local truth-state autonomy launch:
  `USE_TRUTH_STATE_AUTONOMY=true`. That launch keeps the ROSplane
  controller/path stack and mission handling, but feeds it from
  `/sim/rosplane/state` instead of the stock EKF. The stock EKF path remains an
  explicit parity/debug path by overriding `USE_TRUTH_STATE_AUTONOMY=false`.
- A follow-up quadrotor visual rerun exposed a cross-airframe startup hazard:
  after fixed-wing testing, the default Voloxide SIL param store still contained
  fixed-wing secondary mixer state. The quadrotor script now uses
  `/tmp/voloxide_roscopter_sim.params`, and the fixed-wing script uses
  `/tmp/voloxide_rosplane_sim.params`, so plane and quad demos cannot inherit
  each other's persisted mixer/airframe settings.
- The same rerun also exposed a script regression from bypassing ROScopter's
  launch files: the estimator was started directly without the usual
  `imu:=/imu/data` remap. ROSflight publishes `/imu/data`, so the estimator was
  waiting on `/imu`, `/estimated_state` stayed at the origin, and `/command`
  stayed zero. `scripts/run_voloxide_waypoint_demo.zsh` now restores that remap.

## Fixed-Wing Estimator Input Parity Correction

Status: fixed in the Voloxide SIL board shim, validation in progress.

The earlier secondary-mixer diagnosis was a real fixed-wing parity bug, but it
was not the whole fixed-wing estimator story. A later Voloxide automated
ground-start run still reproduced catastrophic ROSplane estimator failure at
the first armed update even after the secondary mixer cache matched C.

C/VimFly first-armed behavior:

- `/sim/RC` transitioned to `[1500, 1500, 1000, 1500, 2000, 2000, 1500, 1500]`.
- `/status` reported `armed=true`, `rc_override=3`, `offboard=true`.
- ROSplane sensor rates were C-like:
  - IMU near 400 Hz,
  - barometer and airspeed near 100 Hz,
  - magnetometer near 50 Hz,
  - GNSS near 10 Hz.
- `/estimated_state` stayed finite at first arm.

Voloxide automated pre-fix behavior:

- The RC/status shape at first arm matched C closely.
- The sensor topic counts did not match C: barometer, GNSS, magnetometer,
  airspeed, and IMU all advanced near the firmware loop rate.
- The Voloxide shim was copying cached simulator sensor messages into every
  `VoloxideFfiSensorSnapshot` and assigning each one the current FCU time. The
  Rust FFI therefore accepted stale simulator samples as fresh data because
  each stale sample appeared to have a new timestamp.

Fix:

- `ros2/voloxide_sil_board_shim/src/voloxide_sil_board.cpp` now passes through
  the original simulator message header timestamps for IMU, magnetometer,
  barometer, GNSS, airspeed, range, and battery.
- The shim now consumes each sensor availability flag after copying it into the
  snapshot, matching the C SIL board's `*_has_new_data_available_` read/clear
  pattern.
- RC remains level-sensitive, matching the C board's `rc_read()` behavior when
  an RC publisher is present.

Post-fix evidence, 2026-05-20:

- `colcon build --packages-select voloxide_sil_board_shim --cmake-args
  -DCMAKE_BUILD_TYPE=RelWithDebInfo` passed.
- A headless Voloxide fixed-wing ground-start run reached `Autopilot ARMED`,
  `Returned to computer control`, and `demo running` without the first-armed
  non-finite estimator cascade.
- Post-fix topic rates matched the C/VimFly sensor cadence:
  - `/imu/data`: about 400 Hz,
  - `/baro`: about 100 Hz,
  - `/airspeed`: about 100 Hz,
  - `/magnetometer`: about 50 Hz,
  - `/gnss`: about 10 Hz,
  - `/sim/pwm_output`: about 400 Hz.
- A sampled `/estimated_state` was finite in flight:
  - `p_d` about `-70.3`,
  - `va` about `17.0`,
  - finite attitude, wind, alpha, beta, and course fields.

Open issue:

- ROSplane's estimator still has local robustness hazards: `Input input_`,
  `Output output`, `lpf_gyro_*`, and `lpf_va_` are not obviously initialized
  before first use, and runtime `ros2 param set` updates do not change low-pass
  alpha until after the first armed `estimate()` path has already used the
  previous alpha. We are not editing ROSplane, so the Voloxide parity target is
  to feed it the same timing and sample freshness profile as the C SIL board.

## Demo Parameter Source of Truth

Status: corrected in launch scripts.

The visual flight demos must not trust persisted Voloxide SIL parameters from
previous tests. Voloxide keeps a local parameter store so param save/load can be
tested, but that persistence is a liability for demo flights: fixed-wing,
quadrotor, and experimental mixer settings can survive into the next run before
the ROSflight initialization launch finishes loading defaults.

The quadrotor and fixed-wing demo scripts now default
`RESET_VOLOXIDE_PARAMS=true`. For `FIRMWARE=voloxide`, they delete the selected
`VOLOXIDE_SIM_PARAM_STORE` before launching firmware, then load the documented
ROSflight parameter file through `multirotor_init_firmware.launch.py` or
`fixedwing_init_firmware.launch.py`. Persisted parameter behavior remains
available only when explicitly requested with `RESET_VOLOXIDE_PARAMS=false`.
