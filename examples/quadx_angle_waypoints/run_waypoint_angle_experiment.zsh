#!/usr/bin/env zsh
set -eo pipefail

EXAMPLE_DIR="${0:A:h}"
VELOXITY="${EXAMPLE_DIR:h:h}"
ROOT="${VELOXITY:h}"
ROSFLIGHT_WS="$ROOT/rosflight/rosflight/workspace"
MISSION="$ROSFLIGHT_WS/src/roscopter/roscopter/params/multirotor_mission.yaml"
ESTIMATOR_PARAMS="$ROSFLIGHT_WS/install/roscopter/share/roscopter/params/estimator.yaml"
MULTIROTOR_PARAMS="$ROSFLIGHT_WS/install/roscopter/share/roscopter/params/multirotor.yaml"

BAG_NAME="takeoff_logs/quadx_waypoint_angle_mode_trial"
DEFAULT_BAG_NAME="$BAG_NAME"
DURATION="90"
PARAMS="$EXAMPLE_DIR/angle_waypoint_baseline.yaml"
AUTO_RELEASE="false"
MAX_PREFLIGHT_PD_ABS="1.0"
FIRMWARE=""
LAUNCH_FIRMWARE=""
USE_RVIZ="true"
INIT_WRITE_DELAY_S="3.0"
SIL_STARTUP_TIMEOUT_S="30"
FINAL_CAL_SETTLE_S="1.0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --firmware)
      FIRMWARE="$2"
      shift 2
      ;;
    --use-rviz)
      USE_RVIZ="$2"
      shift 2
      ;;
    --init-write-delay-s)
      INIT_WRITE_DELAY_S="$2"
      shift 2
      ;;
    --sil-startup-timeout-s)
      SIL_STARTUP_TIMEOUT_S="$2"
      shift 2
      ;;
    --final-cal-settle-s)
      FINAL_CAL_SETTLE_S="$2"
      shift 2
      ;;
    --bag-name)
      BAG_NAME="$2"
      shift 2
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    --params)
      PARAMS="$2"
      shift 2
      ;;
    --auto-release)
      AUTO_RELEASE="true"
      shift
      ;;
    --max-preflight-pd-abs)
      MAX_PREFLIGHT_PD_ABS="$2"
      shift 2
      ;;
    *)
      print -u2 "Unknown argument: $1"
      exit 2
      ;;
  esac
done

case "$FIRMWARE" in
  "")
    LAUNCH_FIRMWARE=""
    ;;
  "c")
    LAUNCH_FIRMWARE="c"
    ;;
  "rust")
    LAUNCH_FIRMWARE="veloxity"
    ;;
  *)
    print -u2 "Unsupported firmware '$FIRMWARE'. Use 'c' or 'rust'."
    exit 2
    ;;
esac

case "$USE_RVIZ" in
  "true"|"false")
    ;;
  *)
    print -u2 "--use-rviz must be 'true' or 'false'."
    exit 2
    ;;
esac

if [[ -n "$FIRMWARE" && "$BAG_NAME" == "$DEFAULT_BAG_NAME" ]]; then
  BAG_NAME="takeoff_logs/quadx_waypoint_angle_mode_${FIRMWARE}"
fi

source /opt/ros/humble/setup.zsh
source "$ROSFLIGHT_WS/install/setup.zsh"
source "$VELOXITY/workspace/install/setup.zsh"

set -u

cd "$VELOXITY"

SIL_PATTERN='multirotor_standalone_sil.launch.py|rviz2|standalone_viz_transcriber|rosflight_sil_manager|(^| )sil_board($| )|veloxity_sil_board|standalone_sensors|rosflight_io|rc.py|multirotor_forces_and_moments|standalone_dynamics'

stale_processes="$(ps -eo pid,args | grep -E 'roscopter (estimator|path_manager|path_planner)|/roscopter/(estimator|path_manager|path_planner)|trajectory_to_angle_command|rviz_waypoint_publisher|ros2 bag record' | grep -v grep || true)"
if [[ -n "$stale_processes" ]]; then
  print -u2 "Refusing to start; stale experiment processes are still running:"
  print -u2 "$stale_processes"
  exit 3
fi

if [[ -n "$FIRMWARE" ]]; then
  stale_sil_processes="$(ps -eo pid,args | grep -E "$SIL_PATTERN" | grep -v grep || true)"
  if [[ -n "$stale_sil_processes" ]]; then
    print -u2 "Refusing to start managed firmware '$FIRMWARE'; visual SIL/RViz processes are already running:"
    print -u2 "$stale_sil_processes"
    print -u2 "Run $EXAMPLE_DIR/clean_slate.zsh first, or omit --firmware to use the existing stack."
    exit 3
  fi
fi

children=()
managed_sil_pid=""

sample_status() {
  timeout 6 ros2 topic echo /status --once 2>/dev/null || true
}

ensure_override_on() {
  local sample
  sample="$(sample_status)"
  if print "$sample" | grep -q "rc_override: 0"; then
    timeout 10 ros2 service call /toggle_override std_srvs/srv/Trigger
  fi
}

ensure_override_off() {
  local sample
  sample="$(sample_status)"
  if ! print "$sample" | grep -q "rc_override: 0"; then
    timeout 10 ros2 service call /toggle_override std_srvs/srv/Trigger
  fi
}

ensure_armed_on() {
  local sample
  sample="$(sample_status)"
  if ! print "$sample" | grep -q "armed: true"; then
    timeout 10 ros2 service call /toggle_arm std_srvs/srv/Trigger
  fi
}

ensure_armed_off() {
  local sample
  sample="$(sample_status)"
  if print "$sample" | grep -q "armed: true"; then
    timeout 10 ros2 service call /toggle_arm std_srvs/srv/Trigger
  fi
}

require_status_field() {
  local pattern="$1"
  local message="$2"
  local sample
  sample="$(sample_status)"
  print "$sample"
  if ! print "$sample" | grep -q "$pattern"; then
    print -u2 "$message"
    return 1
  fi
}

check_preflight_estimator() {
  local sample pd_abs
  sample="$(timeout 6 ros2 topic echo /estimated_state --once 2>/dev/null || true)"
  print "$sample"
  pd_abs="$(print "$sample" | awk '/^p_d:/ {v=$2; if (v < 0) v=-v; print v; exit}')"
  if [[ -z "$pd_abs" ]]; then
    print -u2 "Could not read /estimated_state.p_d for preflight check."
    return 1
  fi
  awk -v value="$pd_abs" -v limit="$MAX_PREFLIGHT_PD_ABS" 'BEGIN { exit !(value <= limit) }'
}

calibrate_on_current_ground_state() {
  print "Settling sensors for $FINAL_CAL_SETTLE_S seconds before final ground calibration..."
  sleep "$FINAL_CAL_SETTLE_S"

  print "Final ground calibration: IMU..."
  timeout 10 ros2 service call /calibrate_imu std_srvs/srv/Trigger

  print "Final ground calibration: baro..."
  timeout 10 ros2 service call /calibrate_baro std_srvs/srv/Trigger

  print "Final ground calibration status:"
  require_status_field "armed: false" "Vehicle is armed after final ground calibration; aborting."
  require_status_field "failsafe: false" "Vehicle is in failsafe after final ground calibration; aborting."
  require_status_field "error_code: 0" "Firmware error_code is nonzero after final ground calibration; aborting."
}

wait_for_managed_sil() {
  local deadline node_list expected_node

  if [[ "$FIRMWARE" == "c" ]]; then
    expected_node="/sil_board"
  else
    expected_node="/veloxity_sil_board"
  fi

  deadline=$((SECONDS + SIL_STARTUP_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    node_list="$(ros2 node list 2>/dev/null || true)"
    if print "$node_list" | grep -q "/rosflight_io" &&
       print "$node_list" | grep -q "$expected_node"; then
      return 0
    fi
    sleep 1
  done

  print -u2 "Timed out waiting for managed SIL stack with $expected_node and /rosflight_io."
  print -u2 "Current ROS nodes:"
  ros2 node list 2>/dev/null || true
  return 1
}

start_managed_sil() {
  print "Starting visual SIL/RViz with firmware=$FIRMWARE use_rviz=$USE_RVIZ..."
  ros2 launch veloxity_sil_board_shim multirotor_standalone_sil.launch.py \
    firmware:="$LAUNCH_FIRMWARE" \
    use_rviz:="$USE_RVIZ" &
  managed_sil_pid="$!"

  wait_for_managed_sil
  timeout 10 ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState

  print "Initializing firmware/baro..."
  ros2 launch veloxity_sil_board_shim veloxity_multirotor_init_firmware.launch.py \
    write_delay_s:="$INIT_WRITE_DELAY_S"

  print "Managed firmware status:"
  require_status_field "armed: false" "Vehicle is armed after firmware init; aborting."
  require_status_field "failsafe: false" "Vehicle is in failsafe after firmware init; aborting."
  local sample
  sample="$(sample_status)"
  print "$sample"
  if ! print "$sample" | grep -q "error_code: 0"; then
    print -u2 "Firmware error_code is nonzero after init; continuing until experiment mixer setup."
  fi
}

stop_children() {
  for pid in "${children[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -INT "$pid" >/dev/null 2>&1
    fi
  done
  sleep 1
  for pid in "${children[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" >/dev/null 2>&1
    fi
  done
  sleep 1
  for pid in "${children[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -KILL "$pid" >/dev/null 2>&1
    fi
  done
  for pid in "${children[@]}"; do
    wait "$pid" >/dev/null 2>&1 || true
  done
}

stop_managed_sil() {
  if [[ -n "$managed_sil_pid" ]] && kill -0 "$managed_sil_pid" >/dev/null 2>&1; then
    kill -INT "$managed_sil_pid" >/dev/null 2>&1 || true
    sleep 1
  fi
  if [[ -n "$managed_sil_pid" ]] && kill -0 "$managed_sil_pid" >/dev/null 2>&1; then
    kill -TERM "$managed_sil_pid" >/dev/null 2>&1 || true
    sleep 1
  fi
  if [[ -n "$managed_sil_pid" ]]; then
    wait "$managed_sil_pid" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  set +e
  stop_children
  ensure_override_on >/dev/null 2>&1 || true
  ensure_armed_off >/dev/null 2>&1 || true
  timeout 10 ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState >/dev/null 2>&1 || true
  timeout 10 ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: PRIMARY_MIXER, value: 11.0}" >/dev/null 2>&1 || true
  timeout 10 ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: USE_MOTOR_PARAM, value: 1.0}" >/dev/null 2>&1 || true
  stop_managed_sil
}
trap cleanup EXIT INT TERM

if [[ -n "$FIRMWARE" ]]; then
  start_managed_sil
fi

print "Checking ROS graph..."
ros2 node list

print "Switching firmware to canned quad-X for angle-mode experiment..."
ensure_override_on
ensure_armed_off
timeout 10 ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: PRIMARY_MIXER, value: 2.0}"
timeout 10 ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: USE_MOTOR_PARAM, value: 0.0}"
timeout 10 ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState
calibrate_on_current_ground_state
require_status_field "error_code: 0" "Firmware error_code is nonzero after quad-X mixer setup; aborting."

print "Starting ROScopter estimator/path stack..."
ros2 run roscopter estimator --ros-args -r __node:=estimator --params-file "$ESTIMATOR_PARAMS" &
children+=("$!")
ros2 run roscopter path_manager --ros-args -r __node:=path_manager --params-file "$MULTIROTOR_PARAMS" -r estimated_state:=estimated_state &
children+=("$!")
ros2 run roscopter path_planner --ros-args -r __node:=path_planner --params-file "$MULTIROTOR_PARAMS" -r estimated_state:=estimated_state &
children+=("$!")
ros2 run roscopter_gcs rviz_waypoint_publisher &
children+=("$!")
python3 "$EXAMPLE_DIR/trajectory_to_angle_command.py" --ros-args --params-file "$PARAMS" &
children+=("$!")

sleep 3

print "Preflight samples:"
timeout 6 ros2 topic echo /command --once
if ! check_preflight_estimator; then
  print -u2 "Estimator altitude is not near ground; run firmware/baro init before flying."
  exit 4
fi
timeout 6 ros2 topic echo /status --once

print "Starting bag: $BAG_NAME"
ros2 bag record -o "$BAG_NAME" \
  /sim/truth_state \
  /estimated_state \
  /trajectory_command \
  /command \
  /status \
  /sim/pwm_output \
  /waypoints &
bag_pid="$!"
children+=("$bag_pid")
sleep 2

print "Arming under override..."
ensure_override_on
ensure_armed_on
require_status_field "armed: true" "Vehicle did not arm; aborting before mission load."

print "Final pre-mission checks:"
timeout 6 ros2 topic echo /status --once
timeout 6 ros2 topic echo /sim/pwm_output --once
timeout 6 ros2 topic echo /command --once

if [[ "$AUTO_RELEASE" == "true" || ! -t 0 ]]; then
  print "Auto-release enabled; loading mission and releasing override immediately."
else
  print "Press Enter to load mission and release override, or Ctrl-C to abort."
  read -r
fi

print "Loading mission..."
timeout 10 ros2 service call /path_planner/load_mission_from_file rosflight_msgs/srv/ParamFile "{filename: $MISSION}"
sleep 0.2
ensure_override_off
require_status_field "rc_override: 0" "Override did not release; aborting before flight."
print "Flying for $DURATION seconds..."
sleep "$DURATION"

print "Stopping bag and cleaning up..."
kill -INT "$bag_pid"
wait "$bag_pid" || true
stop_children

print "Experiment bag:"
ros2 bag info "$BAG_NAME"
