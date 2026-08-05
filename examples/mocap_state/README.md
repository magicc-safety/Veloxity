# Mocap State Publisher

`mocap_state_publisher.py` is an alternative to the ROScopter estimator. Mocap
is authoritative for position and orientation. Firmware `/imu/data`
acceleration predicts velocity between mocap corrections, and firmware
`/attitude` supplies fresh body angular rates. Neither source can move the
absolute mocap reference. The node never publishes `/external_attitude` and
does not alter the firmware estimator or `rosflight_io`.

Run exactly one state source:

```text
v_start_estimator  -> ROScopter EKF -> /estimated_state
v_start_mocap      -> this node     -> /estimated_state
```

The node detects another publisher on `/estimated_state`, latches a fatal
conflict, and stops publishing.

## Configure the transforms

Copy or override `mocap-state.yaml` for the room and vehicle. The incoming pose
is interpreted as the marker frame expressed in the mocap room frame.

- `room_to_ned_quaternion_xyzw` rotates room-frame vectors into NED.
- `body_to_marker_quaternion_xyzw` rotates body-frame vectors into marker axes.
- `body_origin_in_marker_m` is the body origin's position expressed in marker
  axes.

The published quaternion is body-to-NED. This room's position axes are
`(x, y, z) = (north, up, east)`, so its configured room-to-NED quaternion is a
-90 degree X rotation and produces `(p_n, p_e, p_d) = (x, z, -y)`. The marker
basis uses the inverse fixed rotation because an approximately identity input
pose represents a level, north-facing vehicle. Recalibrate
`body_to_marker_quaternion_xyzw` if the mocap rigid-body definition or marker
mount changes. The real-room YAML includes the roll/pitch correction measured
during `fourth_autonomy.csv`; its apparent firmware yaw offset is deliberately
excluded because firmware yaw zero is not a rigid-mount datum.

The configuration uses the fixed mocap room origin rather than the first
received sample. This makes the separately configured room-bounded mission
independent of the aircraft's startup position. In NED, rising in positive
room Y correctly makes `p_d` more negative.

The node supports `geometry_msgs/msg/PoseStamped` and
`geometry_msgs/msg/TransformStamped`. Set `mocap_message_type` to the fully
qualified type used by the driver.

`mocap_qos_depth` defaults to 50. At 240 Hz this retains about 208 ms of
messages if DDS delivers samples in bursts. The independent mocap-age and
filter-gap limits still stop stale state output; queue depth does not extend
those safety deadlines.

## Run

In a shell where ROS 2, ROSflight, and Veloxity are already sourced:

```bash
python3 examples/mocap_state/mocap_state_publisher.py \
  --ros-args --params-file examples/mocap_state/mocap-state.yaml
```

A matching interactive-shell helper is:

```zsh
v_start_mocap() {
  python3 "$VELOXITY_ROOT/examples/mocap_state/mocap_state_publisher.py" \
    --ros-args --params-file "$MOCAP_STATE"
}
```

Set `MOCAP_STATE` to the airframe-local copy of `mocap-state.yaml`. The existing
`v_start_estimator` command remains unchanged. Either command can run in the
existing `estimator` Screen window.

The node publishes its latched validity state on
`/mocap_state_publisher/tracking_valid`. A zero or stale mocap timestamp,
non-finite pose, excessive position innovation, excessive orientation jump,
an exactly frozen pose, stale required IMU, or competing estimated-state
publisher makes validity false and stops state publication.

## Filter behavior

The three NED position axes use an acceleration-aided alpha-beta filter:

```text
predicted_position = position + velocity * dt + 0.5 * acceleration * dt^2
predicted_velocity = velocity + acceleration * dt
residual = measurement - predicted_position
position = predicted_position + alpha * residual
velocity = predicted_velocity + beta / dt * residual
```

The IMU reports body-frame specific force. The node aligns its orientation to
the current mocap attitude, rotates specific force into NED, restores gravity,
and low-pass filters the result. Mocap position residuals continuously bound
accelerometer drift. If `use_firmware_imu` is true, stale IMU data is a fault;
the node does not silently fall back to the laggier constant-velocity path.

The fourth-hover replay reduced the observed velocity phase offset from about
180--220 ms to approximately 9--26 ms. This is replay evidence, not permission
to fly without a restrained, RC-ready validation hover.

Samples whose source timestamps are separated by less than
`minimum_filter_dt_ms` are coalesced. This prevents batched VRPN messages with
microsecond-scale timestamp intervals from amplifying ordinary position noise
through the `beta / dt` velocity correction. They still refresh transport
freshness, but only meaningful source-time intervals advance and publish the
filtered state.

`max_identical_pose_age_ms` independently detects the failure mode where VRPN
keeps advancing timestamps but repeats the exact same pose. A frozen pose stops
publication and cannot be bridged. When pose motion resumes, the filter resets
and must accumulate `minimum_valid_samples` before validity returns.

The enabled short-gap bridge is limited by `max_bridge_age_ms`. It uses fresh
firmware acceleration and relative attitude only while the last real pose is
temporarily delayed. Longer gaps, stale firmware data, and frozen-pose faults
stop `/estimated_state`.

The defaults are starting values, not flight-qualified gains. Record the raw
mocap topic and `/estimated_state`, then tune velocity noise and lag before
releasing RC override.

## Parallel validation

Before allowing this node to own the flight state, run the ordinary estimator
and remap this node's output for comparison:

```bash
python3 examples/mocap_state/mocap_state_publisher.py \
  --ros-args \
  --params-file examples/mocap_state/mocap-state.yaml \
  -r /estimated_state:=/mocap_estimated_state
```

Move the disarmed aircraft through the capture volume and verify NED axes,
attitude signs, body velocity, source timing, jumps, and stale-data behavior.

## Synthetic desktop mocap

`synthetic_mocap_publisher.py` subscribes to the standalone ROSflight
simulator's actual `/sim/truth_state`, converts its NED pose into the mocap
room's `(north, up, east)` convention, and publishes room-frame `PoseStamped`
messages on the same `/mocap/vehicle/pose` topic used in the real room. Never
run it at the same time as the real mocap source. It never copies
`/trajectory_command`; controller and dynamics tracking errors remain present.

In separate sourced shells, run:

```bash
python3 examples/mocap_state/synthetic_mocap_publisher.py \
  --ros-args --params-file examples/mocap_state/synthetic-mocap.yaml

python3 examples/mocap_state/mocap_state_publisher.py \
  --ros-args --params-file examples/mocap_state/synthetic-mocap.yaml
```

The synthetic source resamples fresh simulator truth at 220 Hz. It stops only
for stale or invalid simulator truth. Leaving the physical room produces a
warning but does not interrupt state feedback; the mission retains its separate
two-foot waypoint buffer. RViz may display
`/mocap/vehicle/pose` directly with fixed frame `world`; the existing
standalone visualization continues to show the same underlying dynamics truth.
