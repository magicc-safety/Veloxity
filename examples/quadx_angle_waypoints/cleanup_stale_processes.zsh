#!/usr/bin/env zsh
set -euo pipefail

PATTERN='roscopter (estimator|path_manager|path_planner)|/roscopter/(estimator|path_manager|path_planner)|trajectory_to_angle_command|rviz_waypoint_publisher|ros2 bag record'

matches="$(ps -eo pid,args | grep -E "$PATTERN" | grep -v grep || true)"
if [[ -z "$matches" ]]; then
  print "No stale quad-X angle waypoint example processes found."
  exit 0
fi

print "Stopping stale quad-X angle waypoint example processes:"
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
