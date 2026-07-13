#!/usr/bin/env zsh
set -euo pipefail

PATTERN='roscopter (estimator|path_manager|path_planner|trajectory_follower|controller)|/roscopter/(estimator|path_manager|path_planner|trajectory_follower|controller)|trajectory_velocity_adapter|thrust_to_throttle_adapter|rviz_waypoint_publisher|ros2 bag record'

matches="$(ps -eo pid,args | grep -E "$PATTERN" | grep -v grep || true)"
if [[ -z "$matches" ]]; then
  print "No stale upstream-angle waypoint experiment processes found."
  exit 0
fi

print "Stopping stale upstream-angle waypoint experiment processes:"
print "$matches"

pids=("${(@f)$(print "$matches" | awk '{print $1}')}")
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

print "Done."
