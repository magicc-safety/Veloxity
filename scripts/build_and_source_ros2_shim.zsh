#!/usr/bin/env zsh
emulate -L zsh
set -euo pipefail

if [[ "${ZSH_EVAL_CONTEXT:-}" == *:file ]]; then
  _voloxide_sourced=true
else
  _voloxide_sourced=false
fi

SCRIPT_DIR="${${(%):-%x}:A:h}"
VOLOXIDE_ROOT="${VOLOXIDE_ROOT:-${SCRIPT_DIR:h}}"
VOLOXIDE_WORKSPACE="${VOLOXIDE_WORKSPACE:-${VOLOXIDE_ROOT}/workspace}"
mkdir -p "${VOLOXIDE_WORKSPACE}"

COLCON_BUILD_BASE="${VOLOXIDE_COLCON_BUILD_BASE:-${VOLOXIDE_WORKSPACE}/build}"
COLCON_INSTALL_BASE="${VOLOXIDE_COLCON_INSTALL_BASE:-${VOLOXIDE_WORKSPACE}/install}"
COLCON_LOG_BASE="${VOLOXIDE_COLCON_LOG_BASE:-${VOLOXIDE_WORKSPACE}/log}"
SHIM_PACKAGE_PATH="${VOLOXIDE_SHIM_PACKAGE_PATH:-${VOLOXIDE_ROOT}/sim/ros2/voloxide_sil_board_shim}"

if [[ ! -d "${SHIM_PACKAGE_PATH}" ]]; then
  print -u2 "Voloxide ROS 2 shim package not found: ${SHIM_PACKAGE_PATH}"
  if [[ "${_voloxide_sourced}" == "true" ]]; then return 1; else exit 1; fi
fi

if ! command -v cargo >/dev/null 2>&1; then
  print -u2 "cargo is required to build the Voloxide simulator static library."
  if [[ "${_voloxide_sourced}" == "true" ]]; then return 1; else exit 1; fi
fi

if ! command -v colcon >/dev/null 2>&1; then
  print -u2 "colcon is required to build voloxide_sil_board_shim."
  if [[ "${_voloxide_sourced}" == "true" ]]; then return 1; else exit 1; fi
fi

print -P "%F{cyan}building Voloxide simulator static library%f"
(
  cd "${VOLOXIDE_ROOT}"
  cargo xtask build-sim-lib
)

print -P "%F{cyan}building ROS 2 shim package%f"
(
  cd "${VOLOXIDE_WORKSPACE}"
  colcon --log-base "${COLCON_LOG_BASE}" build \
    --base-paths "${SHIM_PACKAGE_PATH}" \
    --build-base "${COLCON_BUILD_BASE}" \
    --install-base "${COLCON_INSTALL_BASE}" \
    --packages-select voloxide_sil_board_shim
)

if [[ ! -f "${COLCON_INSTALL_BASE}/setup.zsh" ]]; then
  print -u2 "colcon install overlay was not created: ${COLCON_INSTALL_BASE}/setup.zsh"
  if [[ "${_voloxide_sourced}" == "true" ]]; then return 1; else exit 1; fi
fi

_voloxide_restore_nounset=false
if [[ -o nounset ]]; then
  _voloxide_restore_nounset=true
fi
set +u
source "${COLCON_INSTALL_BASE}/setup.zsh"
if [[ "${_voloxide_restore_nounset}" == "true" ]]; then
  set -u
fi

print -P "%F{green}Voloxide ROS 2 shim built and sourced.%f"
print "overlay: ${COLCON_INSTALL_BASE}/setup.zsh"

if [[ "${_voloxide_sourced}" != "true" ]]; then
  print -u2 "Note: run this with 'source' to keep the overlay in your current shell:"
  print -u2 "  source ${(%):-%x}"
fi

if [[ "${_voloxide_sourced}" == "true" ]]; then
  return 0
fi
