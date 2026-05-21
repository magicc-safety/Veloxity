#!/usr/bin/env zsh
set -euo pipefail

SCRIPT_DIR="${${(%):-%x}:A:h}"
VOLOXIDE_ROOT="${SCRIPT_DIR:h}"
PROJECT_ROOT="${VOLOXIDE_ROOT:h}"

cd "${PROJECT_ROOT}"

source "${PROJECT_ROOT}/scripts/source_rosflight_env.zsh"
source "${PROJECT_ROOT}/install/setup.zsh"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/rosflight_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export VOLOXIDE_SIM_PARAM_DIR="${VOLOXIDE_SIM_PARAM_DIR:-${VOLOXIDE_ROOT}/target/voloxide-runtime/rosplane}"
VOLOXIDE_SIM_PARAM_STORE="${VOLOXIDE_SIM_PARAM_DIR}/voloxide_sim.params"
export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"

MISSION_FILE="${MISSION_FILE:-${PROJECT_ROOT}/workspace/src/rosplane/rosplane/missions/fixedwing_mission.yaml}"
DYNAMICS_PARAM_FILE="${DYNAMICS_PARAM_FILE:-${PROJECT_ROOT}/workspace/src/rosflight_ros_pkgs/rosflight_sim/params/anaconda_dynamics.yaml}"
WAYPOINT_MARKER_SCRIPT="${WAYPOINT_MARKER_SCRIPT:-${PROJECT_ROOT}/Voloxide/scripts/rosplane_waypoint_markers.py}"
GRAPH_WAIT_SECONDS="${GRAPH_WAIT_SECONDS:-6}"
ZENOH_STARTUP_SECONDS="${ZENOH_STARTUP_SECONDS:-5}"
ARM_WAIT_SECONDS="${ARM_WAIT_SECONDS:-2}"
USE_VIMFLY="${USE_VIMFLY:-true}"
USE_TRUTH_STATE_AUTONOMY="${USE_TRUTH_STATE_AUTONOMY:-true}"
USE_ROSPLANE_GCS="${USE_ROSPLANE_GCS:-false}"
USE_STANDALONE_RVIZ="${USE_STANDALONE_RVIZ:-true}"
USE_WAYPOINT_VIZ="${USE_WAYPOINT_VIZ:-true}"
MANUAL_TAKEOFF_BEFORE_ROSPLANE="${MANUAL_TAKEOFF_BEFORE_ROSPLANE:-true}"
RESTART_ZENOH="${RESTART_ZENOH:-true}"
FIRMWARE="${FIRMWARE:-voloxide}"
RESET_VOLOXIDE_PARAMS="${RESET_VOLOXIDE_PARAMS:-true}"
ESTIMATOR_GYRO_CUTOFF_FREQ="${ESTIMATOR_GYRO_CUTOFF_FREQ:-100000.0}"
ESTIMATOR_AIRSPEED_CUTOFF_FREQ="${ESTIMATOR_AIRSPEED_CUTOFF_FREQ:-100000.0}"
ESTIMATOR_INCLINATION="${ESTIMATOR_INCLINATION:-67.0}"
ESTIMATOR_DECLINATION="${ESTIMATOR_DECLINATION:-11.0}"
WAYPOINTS_TO_PUBLISH_AT_START="${WAYPOINTS_TO_PUBLISH_AT_START:-100}"
RC_HANDOFF_RELEASE_AIRSPEED="${RC_HANDOFF_RELEASE_AIRSPEED:-17.0}"
RC_HANDOFF_RELEASE_DOWN_POSITION="${RC_HANDOFF_RELEASE_DOWN_POSITION:--70.0}"
if [[ -z "${ROSPLANE_START_AIRSPEED:-}" ]]; then
  if [[ "${USE_VIMFLY}" == "true" ]]; then
    ROSPLANE_START_AIRSPEED="${INITIAL_AIRSPEED:-0.0}"
  elif [[ "${USE_TRUTH_STATE_AUTONOMY}" == "true" ]]; then
    ROSPLANE_START_AIRSPEED=0.1
  else
    ROSPLANE_START_AIRSPEED="${RC_HANDOFF_RELEASE_AIRSPEED}"
  fi
fi
if [[ -z "${ROSPLANE_START_DOWN_POSITION:-}" ]]; then
  if [[ "${USE_VIMFLY}" == "true" ]]; then
    ROSPLANE_START_DOWN_POSITION="${INITIAL_DOWN_POSITION:-0.0}"
  elif [[ "${USE_TRUTH_STATE_AUTONOMY}" == "true" ]]; then
    ROSPLANE_START_DOWN_POSITION=0.0
  else
    ROSPLANE_START_DOWN_POSITION="${RC_HANDOFF_RELEASE_DOWN_POSITION}"
  fi
fi
INITIAL_AIRSPEED="${INITIAL_AIRSPEED:-0.0}"
INITIAL_DOWN_POSITION="${INITIAL_DOWN_POSITION:-0.0}"
if [[ -z "${RC_HANDOFF_SEED_RELEASE_STATE:-}" ]]; then
  if [[ "${USE_TRUTH_STATE_AUTONOMY}" == "true" ]]; then
    RC_HANDOFF_SEED_RELEASE_STATE=true
  else
    RC_HANDOFF_SEED_RELEASE_STATE=false
  fi
fi
SIM_PROCESS_PATTERN='ros2 launch voloxide_sil_board_shim fixedwing_standalone_sil.launch.py|ros2 launch rosflight_sim standalone_sim.launch.py|ros2 launch rosplane_sim sim.launch.py|ros2 launch rosplane_gcs rosplane_gcs.launch.py|rviz2|rosflight_sil_manager|voloxide_sil_board|standalone_sensors|rosflight_io|rc.py|fixedwing_forces_and_moments|standalone_dynamics|standalone_viz_transcriber|controller|estimator|path_follower|path_manager|path_planner|rosplane_truth|rviz_waypoint_publisher|rosplane_waypoint_markers.py|static_transform_publisher'

PIDS=()

run_bg() {
  print -P "%F{cyan}starting:%f $*"
  "$@" &
  PIDS+=("$!")
  sleep 1
}

call_service() {
  print -P "%F{cyan}calling:%f $*"
  "$@"
}

wait_for_zenoh_router() {
  print -P "%F{cyan}waiting ${ZENOH_STARTUP_SECONDS}s for rmw_zenohd startup%f"
  sleep "${ZENOH_STARTUP_SECONDS}"
  if ! pgrep -f '/rmw_zenoh_cpp/rmw_zenohd|ros2 run rmw_zenoh_cpp rmw_zenohd' >/dev/null; then
    print -P "%F{red}rmw_zenohd is not running; aborting before launching ROS nodes%f" >&2
    exit 1
  fi
}

seed_sim_state() {
  local airspeed="${1}"
  local down_position="${2}"
  local state_msg
  state_msg="{state: {pose: {position: {x: 0.0, y: 0.0, z: ${down_position}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: ${airspeed}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, acceleration: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
  call_service ros2 service call /dynamics/set_sim_state rosflight_msgs/srv/SetSimState "${state_msg}"
  ros2 topic pub --once /sim/truth_state rosflight_msgs/msg/SimState \
    "{pose: {position: {x: 0.0, y: 0.0, z: ${down_position}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: ${airspeed}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, acceleration: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
}

seed_release_state_if_enabled() {
  if [[ "${RC_HANDOFF_SEED_RELEASE_STATE}" == "true" ]]; then
    seed_sim_state "${RC_HANDOFF_RELEASE_AIRSPEED}" "${RC_HANDOFF_RELEASE_DOWN_POSITION}"
  else
    print -P "%F{cyan}leaving dynamics state continuous for stock ROSplane estimator release%f"
  fi
}

cleanup_stale_processes() {
  print -P "%F{yellow}clearing stale fixed-wing demo processes%f"
  pkill -f "${SIM_PROCESS_PATTERN}" 2>/dev/null || true
}

reset_voloxide_param_store_if_enabled() {
  mkdir -p "${VOLOXIDE_SIM_PARAM_DIR}"
  if [[ "${FIRMWARE}" == "voloxide" && "${RESET_VOLOXIDE_PARAMS}" == "true" ]]; then
    print -P "%F{yellow}resetting Voloxide param store before loading ROSflight defaults%f"
    rm -f "${VOLOXIDE_SIM_PARAM_STORE}" "${VOLOXIDE_SIM_PARAM_STORE:r}.tmp"
  fi
}

cleanup() {
  print -P "%F{yellow}stopping demo processes%f"
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  pkill -f "${SIM_PROCESS_PATTERN}" 2>/dev/null || true
  if [[ "${RESTART_ZENOH}" == "true" ]]; then
    pkill -f 'rmw_zenohd' 2>/dev/null || true
  fi
}

on_interrupt() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_interrupt INT TERM

print -P "%F{green}Voloxide ROSplane fixed-wing demo%f"
print "project root: ${PROJECT_ROOT}"
print "mission: ${MISSION_FILE}"
print "dynamics: ${DYNAMICS_PARAM_FILE}"
print "firmware: ${FIRMWARE}"
print "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
print "ROS_LOG_DIR=${ROS_LOG_DIR}"
print "VOLOXIDE_SIM_PARAM_DIR=${VOLOXIDE_SIM_PARAM_DIR}"
print "RESET_VOLOXIDE_PARAMS=${RESET_VOLOXIDE_PARAMS}"
print "ROSplane startup state: airspeed=${ROSPLANE_START_AIRSPEED}, down=${ROSPLANE_START_DOWN_POSITION}"
print "RC release state: airspeed=${RC_HANDOFF_RELEASE_AIRSPEED}, down=${RC_HANDOFF_RELEASE_DOWN_POSITION}, seed=${RC_HANDOFF_SEED_RELEASE_STATE}"

cleanup_stale_processes
reset_voloxide_param_store_if_enabled

if [[ "${RESTART_ZENOH}" == "true" ]]; then
  pkill -f 'rmw_zenohd' 2>/dev/null || true
  run_bg ros2 run rmw_zenoh_cpp rmw_zenohd
  wait_for_zenoh_router
elif pgrep -f 'rmw_zenohd' >/dev/null; then
  print -P "%F{cyan}using existing rmw_zenohd router%f"
else
  run_bg ros2 run rmw_zenoh_cpp rmw_zenohd
  wait_for_zenoh_router
fi

run_bg ros2 launch voloxide_sil_board_shim fixedwing_standalone_sil.launch.py \
  firmware:="${FIRMWARE}" \
  use_rviz:=false \
  use_vimfly:="${USE_VIMFLY}" \
  use_builtin_rc:=true \
  dynamics_param_file:="${DYNAMICS_PARAM_FILE}"

print -P "%F{cyan}waiting for SIL graph discovery%f"
sleep "${GRAPH_WAIT_SECONDS}"

print -P "%F{cyan}seeding fixed-wing calibration state%f"
seed_sim_state "${INITIAL_AIRSPEED}" "${INITIAL_DOWN_POSITION}"

print -P "%F{cyan}loading fixed-wing firmware params and running firmware calibration%f"
ros2 launch rosflight_sim fixedwing_init_firmware.launch.py

print -P "%F{cyan}refreshing fixed-wing dynamics firmware parameter cache%f"
call_service ros2 service call /all_params_received std_srvs/srv/Trigger "{}"
ros2 topic pub --once /status/params_changed std_msgs/msg/Bool "{data: true}"
sleep 1

if [[ "${USE_STANDALONE_RVIZ}" == "true" ]]; then
  print -P "%F{cyan}starting standalone fixed-wing RViz%f"
  run_bg ros2 launch rosflight_sim standalone_sim.launch.py \
    sim_aircraft_file:=common_resource/skyhunter.dae
fi

if [[ "${USE_VIMFLY}" == "true" && "${MANUAL_TAKEOFF_BEFORE_ROSPLANE}" == "true" ]]; then
  print -P "%F{cyan}manual fixed-wing takeoff%f"
  print "Click the VimFly window. Press t once to arm, then use VimFly to take off manually under RC override."
  print "Do not press r yet. Press Enter here only after the aircraft is airborne and under manual control."
  read -r
else
  print -P "%F{cyan}seeding finite ROSplane startup state%f"
  seed_sim_state "${ROSPLANE_START_AIRSPEED}" "${ROSPLANE_START_DOWN_POSITION}"
fi

print -P "%F{cyan}starting ROSplane autonomy stack%f"
if [[ "${USE_TRUTH_STATE_AUTONOMY}" == "true" ]]; then
  run_bg ros2 launch voloxide_sil_board_shim rosplane_truth_state_autonomy.launch.py
else
  run_bg ros2 launch rosplane_sim sim.launch.py
fi

if [[ "${USE_ROSPLANE_GCS}" == "true" ]]; then
  print -P "%F{cyan}starting ROSplane ground-control visualization%f"
  run_bg ros2 launch rosplane_gcs rosplane_gcs.launch.py
elif [[ "${USE_WAYPOINT_VIZ}" == "true" ]]; then
  print -P "%F{cyan}starting waypoint-only marker publisher%f"
  run_bg python3 "${WAYPOINT_MARKER_SCRIPT}"
fi

sleep 3

print -P "%F{cyan}seeding ROSplane estimator startup parameters%f"
if ros2 node list | grep -qx '/estimator'; then
  ros2 param set /estimator gyro_cutoff_freq "${ESTIMATOR_GYRO_CUTOFF_FREQ}"
  ros2 param set /estimator airspeed_cutoff_freq "${ESTIMATOR_AIRSPEED_CUTOFF_FREQ}"
  ros2 param set /estimator inclination "${ESTIMATOR_INCLINATION}"
  ros2 param set /estimator declination "${ESTIMATOR_DECLINATION}"
fi
ros2 param set /path_planner num_waypoints_to_publish_at_start "${WAYPOINTS_TO_PUBLISH_AT_START}"

if [[ "${USE_VIMFLY}" == "true" && "${MANUAL_TAKEOFF_BEFORE_ROSPLANE}" == "true" ]]; then
  print -P "%F{cyan}leaving manually flown state continuous for ROSplane handoff%f"
else
  print -P "%F{cyan}re-seeding finite ROSplane startup state before arming%f"
  seed_sim_state "${ROSPLANE_START_AIRSPEED}" "${ROSPLANE_START_DOWN_POSITION}"
  sleep 1
fi

print -P "%F{cyan}loading fixed-wing mission%f"
ros2 service call /load_mission_from_file rosflight_msgs/srv/ParamFile \
  "{filename: ${MISSION_FILE}}"

if [[ "${USE_VIMFLY}" == "true" ]]; then
  print -P "%F{cyan}arm and release RC override in VimFly%f"
  if [[ "${MANUAL_TAKEOFF_BEFORE_ROSPLANE}" == "true" ]]; then
    print "Keep the aircraft flying manually. ROSplane is now running and the mission is loaded."
  else
    print "Click the VimFly window, press t once to arm, then press Enter here after /status shows armed=true."
    read -r
    print -P "%F{cyan}preparing state before RC override release%f"
    seed_release_state_if_enabled
  fi
  print "Press r once in VimFly to release RC override, then press Enter here after /status shows rc_override=0."
  read -r
else
  print -P "%F{cyan}arming and releasing RC override through simulated-RC services%f"
  call_service ros2 service call /toggle_arm std_srvs/srv/Trigger "{}"
  sleep "${ARM_WAIT_SECONDS}"
  print -P "%F{cyan}preparing state before RC override release%f"
  seed_release_state_if_enabled
  call_service ros2 service call /toggle_override std_srvs/srv/Trigger "{}"
fi

print -P "%F{green}demo running%f"
print "RViz should show the aircraft and ROSplane waypoint markers."
print "Useful monitors:"
print "  ros2 topic hz /command"
print "  ros2 topic hz /sim/pwm_output"
print "  ros2 topic echo /estimated_state"
print "  ros2 topic echo /sim/truth_state"
print "  ros2 topic echo /controller_internals"
print "Press Ctrl-C in this terminal to stop the demo."

wait
