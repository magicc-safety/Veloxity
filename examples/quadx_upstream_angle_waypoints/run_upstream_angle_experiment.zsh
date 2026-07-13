#!/usr/bin/env zsh
set -eo pipefail

EXAMPLE_DIR="${0:A:h}"
VELOXITY="${EXAMPLE_DIR:h:h}"
EXPERIMENT_PARAMS="$EXAMPLE_DIR/upstream_angle_baseline.yaml"
MISSION=""

FIRMWARE="rust"
LAUNCH_FIRMWARE="veloxity"
USE_RVIZ="true"
DURATION="120"
BAG_NAME="takeoff_logs/quadx_upstream_angle_mode_rust"
RECORD_ALL="false"
VELOCITY_FEEDFORWARD="true"
INIT_WRITE_DELAY_S="3.0"
SIL_STARTUP_TIMEOUT_S="30"
FINAL_CAL_SETTLE_S="1.0"
MAX_PREFLIGHT_PD_ABS="1.0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --firmware) FIRMWARE="$2"; shift 2 ;;
    --use-rviz) USE_RVIZ="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --bag-name) BAG_NAME="$2"; shift 2 ;;
    --record-all) RECORD_ALL="$2"; shift 2 ;;
    --velocity-feedforward) VELOCITY_FEEDFORWARD="$2"; shift 2 ;;
    --mission) MISSION="$2"; shift 2 ;;
    --init-write-delay-s) INIT_WRITE_DELAY_S="$2"; shift 2 ;;
    --sil-startup-timeout-s) SIL_STARTUP_TIMEOUT_S="$2"; shift 2 ;;
    --final-cal-settle-s) FINAL_CAL_SETTLE_S="$2"; shift 2 ;;
    --max-preflight-pd-abs) MAX_PREFLIGHT_PD_ABS="$2"; shift 2 ;;
    *) print -u2 "Unknown argument: $1"; exit 2 ;;
  esac
done

case "$FIRMWARE" in
  rust) LAUNCH_FIRMWARE="veloxity" ;;
  c) LAUNCH_FIRMWARE="c" ;;
  *) print -u2 "Unsupported firmware '$FIRMWARE'. Use 'c' or 'rust'."; exit 2 ;;
esac
case "$USE_RVIZ" in
  true|false) ;;
  *) print -u2 "--use-rviz must be 'true' or 'false'."; exit 2 ;;
esac
case "$RECORD_ALL" in
  true|false) ;;
  *) print -u2 "--record-all must be 'true' or 'false'."; exit 2 ;;
esac
case "$VELOCITY_FEEDFORWARD" in
  true|false) ;;
  *) print -u2 "--velocity-feedforward must be 'true' or 'false'."; exit 2 ;;
esac

if ! command -v ros2 >/dev/null 2>&1; then
  print -u2 "ROS 2 is not sourced. Source ROS 2, the ROSflight workspace, and scripts/build_and_source_ros2_shim.zsh first."
  exit 2
fi
ROSCOPTER_PREFIX="$(ros2 pkg prefix roscopter 2>/dev/null)" || {
  print -u2 "The already-sourced ROS environment does not contain the roscopter package."
  exit 2
}
if ! ros2 pkg prefix veloxity_sil_board_shim >/dev/null 2>&1; then
  print -u2 "The Veloxity ROS 2 shim is not sourced. Run: source scripts/build_and_source_ros2_shim.zsh"
  exit 2
fi
[[ -n "$MISSION" ]] || MISSION="$ROSCOPTER_PREFIX/share/roscopter/params/multirotor_mission.yaml"
ESTIMATOR_PARAMS="$ROSCOPTER_PREFIX/share/roscopter/params/estimator.yaml"
MULTIROTOR_PARAMS="$ROSCOPTER_PREFIX/share/roscopter/params/multirotor.yaml"
set -u
cd "$VELOXITY"

EXAMPLE_PATTERN='roscopter (estimator|path_manager|path_planner|trajectory_follower|controller)|/roscopter/(estimator|path_manager|path_planner|trajectory_follower|controller)|trajectory_to_angle_command|trajectory_velocity_adapter|thrust_to_throttle_adapter|rviz_waypoint_publisher|ros2 bag record'
SIL_PATTERN='multirotor_standalone_sil.launch.py|rviz2|standalone_viz_transcriber|rosflight_sil_manager|(^| )sil_board($| )|veloxity_sil_board|standalone_sensors|rosflight_io|rc.py|multirotor_forces_and_moments|standalone_dynamics'

stale="$(ps -eo pid,args | grep -E "$EXAMPLE_PATTERN|$SIL_PATTERN" | grep -v grep || true)"
if [[ -n "$stale" ]]; then
  print -u2 "Refusing to start; experiment or SIL processes are already running:"
  print -u2 "$stale"
  print -u2 "Run $EXAMPLE_DIR/clean_slate.zsh first."
  exit 3
fi

children=()
managed_sil_pid=""

sample_status() {
  timeout 6 ros2 topic echo /status --once 2>/dev/null || true
}

require_status_field() {
  local pattern="$1" message="$2" sample
  sample="$(sample_status)"
  print "$sample"
  if ! print "$sample" | grep -q "$pattern"; then
    print -u2 "$message"
    return 1
  fi
}

wait_for_status_field() {
  local pattern="$1" message="$2" timeout_s="$3" deadline sample
  deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    sample="$(sample_status)"
    if print "$sample" | grep -q "$pattern"; then
      print "$sample"
      return 0
    fi
  done
  print "$sample"
  print -u2 "$message"
  return 1
}

ensure_override_on() {
  local sample="$(sample_status)"
  if print "$sample" | grep -q "rc_override: 0"; then
    timeout 10 ros2 service call /toggle_override std_srvs/srv/Trigger
  fi
}

ensure_override_off() {
  local sample="$(sample_status)"
  if ! print "$sample" | grep -q "rc_override: 0"; then
    timeout 10 ros2 service call /toggle_override std_srvs/srv/Trigger
  fi
}

ensure_armed_on() {
  local sample="$(sample_status)"
  if ! print "$sample" | grep -q "armed: true"; then
    timeout 10 ros2 service call /toggle_arm std_srvs/srv/Trigger
  fi
}

ensure_armed_off() {
  local sample="$(sample_status)"
  if print "$sample" | grep -q "armed: true"; then
    timeout 10 ros2 service call /toggle_arm std_srvs/srv/Trigger
  fi
}

stop_children() {
  for pid in "${children[@]}"; do
    kill -INT "$pid" >/dev/null 2>&1 || true
  done
  sleep 1
  for pid in "${children[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done
  for pid in "${children[@]}"; do
    wait "$pid" >/dev/null 2>&1 || true
  done
}

cleanup() {
  set +e
  stop_children
  ensure_override_on >/dev/null 2>&1 || true
  ensure_armed_off >/dev/null 2>&1 || true
  timeout 10 ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState >/dev/null 2>&1 || true
  timeout 10 ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: PRIMARY_MIXER, value: 11.0}" >/dev/null 2>&1 || true
  timeout 10 ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: USE_MOTOR_PARAM, value: 1.0}" >/dev/null 2>&1 || true
  if [[ -n "$managed_sil_pid" ]] && kill -0 "$managed_sil_pid" >/dev/null 2>&1; then
    kill -INT "$managed_sil_pid" >/dev/null 2>&1 || true
    sleep 1
  fi
  if [[ -n "$managed_sil_pid" ]] && kill -0 "$managed_sil_pid" >/dev/null 2>&1; then
    kill -TERM "$managed_sil_pid" >/dev/null 2>&1 || true
  fi
  [[ -n "$managed_sil_pid" ]] && wait "$managed_sil_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

print "Starting visual SIL/RViz with firmware=$FIRMWARE use_rviz=$USE_RVIZ..."
ros2 launch veloxity_sil_board_shim multirotor_standalone_sil.launch.py \
  firmware:="$LAUNCH_FIRMWARE" use_rviz:="$USE_RVIZ" &
managed_sil_pid="$!"

expected_node="/veloxity_sil_board"
[[ "$FIRMWARE" == "c" ]] && expected_node="/sil_board"
deadline=$((SECONDS + SIL_STARTUP_TIMEOUT_S))
while (( SECONDS < deadline )); do
  nodes="$(ros2 node list 2>/dev/null || true)"
  if print "$nodes" | grep -q "/rosflight_io" && print "$nodes" | grep -q "$expected_node"; then
    break
  fi
  sleep 1
done
nodes="$(ros2 node list 2>/dev/null || true)"
if ! print "$nodes" | grep -q "/rosflight_io" || ! print "$nodes" | grep -q "$expected_node"; then
  print -u2 "Timed out waiting for $expected_node and /rosflight_io."
  exit 4
fi

timeout 10 ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState
print "Initializing firmware/barometer..."
ros2 launch veloxity_sil_board_shim veloxity_multirotor_init_firmware.launch.py \
  write_delay_s:="$INIT_WRITE_DELAY_S"
require_status_field "armed: false" "Vehicle is armed after firmware init."
require_status_field "failsafe: false" "Vehicle is in failsafe after firmware init."

print "Selecting canned quad-X mixer for normalized firmware throttle..."
ensure_override_on
ensure_armed_off
timeout 10 ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: PRIMARY_MIXER, value: 2.0}"
timeout 10 ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: USE_MOTOR_PARAM, value: 0.0}"
timeout 10 ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState
sleep "$FINAL_CAL_SETTLE_S"
timeout 10 ros2 service call /calibrate_imu std_srvs/srv/Trigger
timeout 10 ros2 service call /calibrate_baro std_srvs/srv/Trigger
wait_for_status_field "error_code: 0" "Firmware error_code is nonzero after calibration." 12

print "Starting upstream ROScopter stack with mode adapter..."
ros2 run roscopter estimator --ros-args -r __node:=estimator --params-file "$ESTIMATOR_PARAMS" &
children+=("$!")
ros2 run roscopter path_manager --ros-args -r __node:=path_manager --params-file "$MULTIROTOR_PARAMS" --params-file "$EXPERIMENT_PARAMS" -r estimated_state:=estimated_state &
children+=("$!")
ros2 run roscopter path_planner --ros-args -r __node:=path_planner --params-file "$MULTIROTOR_PARAMS" -r estimated_state:=estimated_state &
children+=("$!")
follower_trajectory_topic="trajectory_command"
if [[ "$VELOCITY_FEEDFORWARD" == "true" ]]; then
  print "Enabling trajectory velocity feed-forward adapter."
  python3 "$EXAMPLE_DIR/trajectory_velocity_adapter.py" --ros-args --params-file "$EXPERIMENT_PARAMS" &
  children+=("$!")
  follower_trajectory_topic="trajectory_command_compensated"
else
  print "Velocity feed-forward disabled; follower consumes the original trajectory command."
fi
ros2 run roscopter trajectory_follower --ros-args -r __node:=trajectory_follower --params-file "$MULTIROTOR_PARAMS" --params-file "$EXPERIMENT_PARAMS" -r estimated_state:=estimated_state -r trajectory_command:="$follower_trajectory_topic" -r high_level_command:=high_level_command_thrust &
children+=("$!")
python3 "$EXAMPLE_DIR/thrust_to_throttle_adapter.py" --ros-args --params-file "$MULTIROTOR_PARAMS" --params-file "$EXPERIMENT_PARAMS" &
children+=("$!")
ros2 run roscopter controller --ros-args -r __node:=controller --params-file "$MULTIROTOR_PARAMS" -r estimated_state:=estimated_state &
children+=("$!")
ros2 run roscopter_gcs rviz_waypoint_publisher &
children+=("$!")

sleep 4
print "Checking graph and preflight state..."
ros2 node list
ros2 topic info /command --verbose
ros2 topic info /high_level_command --verbose
follower_topic_info="$(ros2 topic info "/$follower_trajectory_topic" --verbose)"
print "$follower_topic_info"
follower_publishers="$(print "$follower_topic_info" | awk '/^Publisher count:/ {print $3; exit}')"
follower_subscribers="$(print "$follower_topic_info" | awk '/^Subscription count:/ {print $3; exit}')"
if [[ -z "$follower_publishers" || "$follower_publishers" -lt 1 || \
      -z "$follower_subscribers" || "$follower_subscribers" -lt 1 ]]; then
  print -u2 "/$follower_trajectory_topic is not fully connected " \
    "(publishers=${follower_publishers:-unknown}, subscribers=${follower_subscribers:-unknown})."
  exit 5
fi
state="$(timeout 6 ros2 topic echo /estimated_state --once 2>/dev/null || true)"
print "$state"
pd_abs="$(print "$state" | awk '/^p_d:/ {v=$2; if (v < 0) v=-v; print v; exit}')"
if [[ -z "$pd_abs" ]] || ! awk -v value="$pd_abs" -v limit="$MAX_PREFLIGHT_PD_ABS" 'BEGIN { exit !(value <= limit) }'; then
  print -u2 "Estimator altitude is not near the ground."
  exit 5
fi
require_status_field "error_code: 0" "Firmware is unhealthy before arming."

print "Starting bag: $BAG_NAME"
if [[ "$RECORD_ALL" == "true" ]]; then
  ros2 bag record -a -o "$BAG_NAME" &
else
  ros2 bag record -o "$BAG_NAME" \
    /sim/truth_state /estimated_state /trajectory_command /trajectory_command_compensated \
    /high_level_command_thrust /high_level_command \
    /command /status /sim/pwm_output /waypoints &
fi
bag_pid="$!"
children+=("$bag_pid")
sleep 2

print "Arming under simulated RC override..."
ensure_override_on
ensure_armed_on
require_status_field "armed: true" "Vehicle did not arm."
sleep 1
print "Controller output before mission release:"
timeout 6 ros2 topic echo /command --once

print "Loading mission..."
timeout 10 ros2 service call /path_planner/load_mission_from_file rosflight_msgs/srv/ParamFile "{filename: $MISSION}"
sleep 0.2
ensure_override_off
require_status_field "rc_override: 0" "Override did not release."
print "Flying for $DURATION seconds..."
sleep "$DURATION"

print "Stopping bag and cleaning up..."
kill -INT "$bag_pid"
wait "$bag_pid" || true
stop_children
print "Experiment bag:"
ros2 bag info "$BAG_NAME"
