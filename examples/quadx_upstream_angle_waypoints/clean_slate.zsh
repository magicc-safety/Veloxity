#!/usr/bin/env zsh
set -euo pipefail

EXAMPLE_PATTERN='roscopter (estimator|path_manager|path_planner|trajectory_follower|controller)|/roscopter/(estimator|path_manager|path_planner|trajectory_follower|controller)|trajectory_velocity_adapter|thrust_to_throttle_adapter|rviz_waypoint_publisher|ros2 bag record'
SIL_PATTERN='multirotor_standalone_sil.launch.py|rviz2|standalone_viz_transcriber|rosflight_sil_manager|veloxity_sil_board|standalone_sensors|rosflight_io|rc.py|multirotor_forces_and_moments|standalone_dynamics'

stop_matches() {
  local label="$1"
  local pattern="$2"
  local matches
  matches="$(ps -eo pid,args | grep -E "$pattern" | grep -v grep || true)"
  if [[ -z "$matches" ]]; then
    print "No $label processes found."
    return
  fi

  print "Stopping $label processes:"
  print "$matches"
  local pids=("${(@f)$(print "$matches" | awk '{print $1}')}")
  for pid in "${pids[@]}"; do
    kill -INT "$pid" >/dev/null 2>&1 || true
  done
  sleep 1
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done
  sleep 1
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -KILL "$pid" >/dev/null 2>&1 || true
    fi
  done
}

stop_matches "upstream-angle waypoint experiment" "$EXAMPLE_PATTERN"
stop_matches "visual SIL/RViz" "$SIL_PATTERN"
print "Clean slate complete."
