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

MISSION_FILE="${MISSION_FILE:-${PROJECT_ROOT}/workspace/src/roscopter/roscopter/params/multirotor_mission.yaml}"
RHO_NOT_IN_USE="${RHO_NOT_IN_USE:--1000000.0}"
CALIBRATION_WAIT_SECONDS="${CALIBRATION_WAIT_SECONDS:-4}"

PIDS=()

run_bg() {
  print -P "%F{cyan}starting:%f $*"
  "$@" &
  PIDS+=("$!")
  sleep 1
}

cleanup() {
  print -P "%F{yellow}stopping demo processes%f"
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  pkill -f 'ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py|rviz2|rosflight_sil_manager|voloxide_sil_board|standalone_sensors|rosflight_io|rc.py|multirotor_forces_and_moments|standalone_dynamics|controller|estimator|trajectory_follower|path_manager|path_planner|rviz_waypoint_publisher|standalone_viz_transcriber|static_transform_publisher|rmw_zenohd' 2>/dev/null || true
}

trap cleanup EXIT INT TERM

print -P "%F{green}Voloxide ROScopter waypoint demo%f"
print "project root: ${PROJECT_ROOT}"
print "mission: ${MISSION_FILE}"
print "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
print "ROS_LOG_DIR=${ROS_LOG_DIR}"

run_bg ros2 run rmw_zenoh_cpp rmw_zenohd

run_bg ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py \
  firmware:=voloxide \
  use_rviz:=true

print -P "%F{cyan}waiting for SIL graph discovery%f"
sleep 6

print -P "%F{cyan}loading firmware params and running firmware calibration%f"
ros2 launch rosflight_sim multirotor_init_firmware.launch.py
ros2 service call /calibrate_imu std_srvs/srv/Trigger

print -P "%F{cyan}starting estimator with rho left as NOT_IN_USE%f"
run_bg ros2 run roscopter estimator \
  --ros-args \
  --params-file "${PROJECT_ROOT}/workspace/install/roscopter/share/roscopter/params/estimator.yaml" \
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
