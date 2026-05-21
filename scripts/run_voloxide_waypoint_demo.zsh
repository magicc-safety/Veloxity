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
export VOLOXIDE_SIM_PARAM_DIR="${VOLOXIDE_SIM_PARAM_DIR:-${VOLOXIDE_ROOT}/target/voloxide-runtime/roscopter}"
voloxide_sim_param_file="${VOLOXIDE_SIM_PARAM_DIR}/voloxide_sim.params"

RESET_VOLOXIDE_PARAMS="${RESET_VOLOXIDE_PARAMS:-true}"
MISSION_FILE="${MISSION_FILE:-${PROJECT_ROOT}/workspace/src/roscopter/roscopter/params/multirotor_mission.yaml}"
RHO_NOT_IN_USE="${RHO_NOT_IN_USE:--1000000.0}"
FIRMWARE_FILTER_USE_ACC="${FIRMWARE_FILTER_USE_ACC:-0}"
CALIBRATION_WAIT_SECONDS="${CALIBRATION_WAIT_SECONDS:-4}"
RESTART_ZENOH="${RESTART_ZENOH:-true}"
USE_RVIZ="${USE_RVIZ:-true}"
SIM_PROCESS_PATTERN='ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py|rviz2|rosflight_sil_manager|voloxide_sil_board|standalone_sensors|rosflight_io|rc.py|multirotor_forces_and_moments|standalone_dynamics|standalone_viz_transcriber|controller|estimator|trajectory_follower|path_manager|path_planner|rviz_waypoint_publisher|static_transform_publisher'

PIDS=()

run_bg() {
  print -P "%F{cyan}starting:%f $*"
  "$@" &
  PIDS+=("$!")
  sleep 1
}

cleanup_stale_processes() {
  print -P "%F{yellow}clearing stale multirotor demo processes%f"
  pkill -f "${SIM_PROCESS_PATTERN}" 2>/dev/null || true
}

reset_voloxide_param_store_if_enabled() {
  mkdir -p "${VOLOXIDE_SIM_PARAM_DIR}"
  if [[ "${RESET_VOLOXIDE_PARAMS}" == "true" ]]; then
    print -P "%F{yellow}resetting Voloxide param store before loading ROSflight defaults%f"
    rm -f "${voloxide_sim_param_file}" "${voloxide_sim_param_file:r}.tmp"
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

trap cleanup EXIT INT TERM

print -P "%F{green}Voloxide ROScopter waypoint demo%f"
print "project root: ${PROJECT_ROOT}"
print "mission: ${MISSION_FILE}"
print "firmware: voloxide"
print "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
print "ROS_LOG_DIR=${ROS_LOG_DIR}"
print "VOLOXIDE_SIM_PARAM_DIR=${VOLOXIDE_SIM_PARAM_DIR}"
print "RESET_VOLOXIDE_PARAMS=${RESET_VOLOXIDE_PARAMS}"
print "USE_RVIZ=${USE_RVIZ}"

cleanup_stale_processes
reset_voloxide_param_store_if_enabled

if [[ "${RESTART_ZENOH}" == "true" ]]; then
  pkill -f 'rmw_zenohd' 2>/dev/null || true
  run_bg ros2 run rmw_zenoh_cpp rmw_zenohd
elif pgrep -f 'rmw_zenohd' >/dev/null; then
  print -P "%F{cyan}using existing rmw_zenohd router%f"
else
  run_bg ros2 run rmw_zenoh_cpp rmw_zenohd
fi

run_bg ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py \
  firmware:=voloxide \
  use_rviz:="${USE_RVIZ}"

print -P "%F{cyan}waiting for SIL graph discovery%f"
sleep 6

print -P "%F{cyan}loading firmware params and running firmware calibration%f"
ros2 launch rosflight_sim multirotor_init_firmware.launch.py
ros2 service call /calibrate_imu std_srvs/srv/Trigger
print -P "%F{cyan}configuring firmware attitude estimator for external ROScopter attitude%f"
ros2 service call /param_set rosflight_msgs/srv/ParamSet \
  "{name: FILT_USE_ACC, value: ${FIRMWARE_FILTER_USE_ACC}}"

print -P "%F{cyan}starting estimator with rho left as NOT_IN_USE%f"
run_bg ros2 run roscopter estimator \
  --ros-args \
  --params-file "${PROJECT_ROOT}/workspace/install/roscopter/share/roscopter/params/estimator.yaml" \
  -r imu:=/imu/data \
  -p hotstart_estimator:=false \
  -p rho:="${RHO_NOT_IN_USE}"

sleep 2

print -P "%F{cyan}arming and waiting for stationary barometer calibration%f"
ros2 service call /toggle_arm std_srvs/srv/Trigger
sleep "${CALIBRATION_WAIT_SECONDS}"

print -P "%F{cyan}starting ROScopter autonomy nodes%f"
run_bg ros2 run roscopter controller \
  --ros-args --params-file "${PROJECT_ROOT}/workspace/install/roscopter/share/roscopter/params/multirotor.yaml"
run_bg ros2 run roscopter trajectory_follower \
  --ros-args --params-file "${PROJECT_ROOT}/workspace/install/roscopter/share/roscopter/params/multirotor.yaml"
run_bg ros2 run roscopter path_manager \
  --ros-args --params-file "${PROJECT_ROOT}/workspace/install/roscopter/share/roscopter/params/multirotor.yaml"
run_bg ros2 run roscopter path_planner \
  --ros-args --params-file "${PROJECT_ROOT}/workspace/install/roscopter/share/roscopter/params/multirotor.yaml"
run_bg ros2 run roscopter_gcs rviz_waypoint_publisher

sleep 3

print -P "%F{cyan}configuring path manager and loading mission%f"
ros2 param set /path_manager hold_last true
ros2 service call /path_planner/load_mission_from_file rosflight_msgs/srv/ParamFile \
  "{filename: ${MISSION_FILE}}"

print -P "%F{cyan}releasing RC override for offboard waypoint following%f"
ros2 service call /toggle_override std_srvs/srv/Trigger

print -P "%F{green}demo running%f"
print "RViz should show the aircraft and waypoint markers."
print "Useful monitors:"
print "  ros2 topic hz /command"
print "  ros2 topic hz /sim/pwm_output"
print "  ros2 topic echo /estimated_state"
print "  ros2 topic echo /sim/truth_state"
print "Press Ctrl-C in this terminal to stop the demo."

wait
