#!/usr/bin/env zsh
set -euo pipefail

SCRIPT_DIR="${${(%):-%x}:A:h}"

export FIRMWARE="${FIRMWARE:-voloxide}"
export USE_VIMFLY="${USE_VIMFLY:-true}"
export USE_TRUTH_STATE_AUTONOMY="${USE_TRUTH_STATE_AUTONOMY:-true}"
export USE_STANDALONE_RVIZ="${USE_STANDALONE_RVIZ:-true}"
export USE_WAYPOINT_VIZ="${USE_WAYPOINT_VIZ:-true}"
export USE_ROSPLANE_GCS="${USE_ROSPLANE_GCS:-false}"
export MANUAL_TAKEOFF_BEFORE_ROSPLANE="${MANUAL_TAKEOFF_BEFORE_ROSPLANE:-true}"
export RESTART_ZENOH="${RESTART_ZENOH:-true}"
export RC_HANDOFF_RELEASE_AIRSPEED="${RC_HANDOFF_RELEASE_AIRSPEED:-17.0}"
export RC_HANDOFF_RELEASE_DOWN_POSITION="${RC_HANDOFF_RELEASE_DOWN_POSITION:--70.0}"

print -P "%F{green}Voloxide ROSplane tutorial demo%f"
print "This follows the ROSplane tutorial flow with Voloxide as the firmware endpoint."
print "The visual tutorial path uses VimFly: calibrate, take off manually, then start the ROSplane controller/path stack from truth-state sim input and release RC override."
print

exec "${SCRIPT_DIR}/run_voloxide_rosplane_demo.zsh"
