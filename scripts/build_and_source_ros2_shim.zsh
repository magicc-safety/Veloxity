#!/usr/bin/env zsh
emulate -L zsh
set -euo pipefail

if [[ "${ZSH_EVAL_CONTEXT:-}" == *:file ]]; then
  _veloxity_sourced=true
else
  _veloxity_sourced=false
fi

SCRIPT_DIR="${${(%):-%x}:A:h}"
VELOXITY_ROOT="${VELOXITY_ROOT:-${SCRIPT_DIR:h}}"
VELOXITY_WORKSPACE="${VELOXITY_WORKSPACE:-${VELOXITY_ROOT}/workspace}"
mkdir -p "${VELOXITY_WORKSPACE}"

COLCON_BUILD_BASE="${VELOXITY_COLCON_BUILD_BASE:-${VELOXITY_WORKSPACE}/build}"
COLCON_INSTALL_BASE="${VELOXITY_COLCON_INSTALL_BASE:-${VELOXITY_WORKSPACE}/install}"
COLCON_LOG_BASE="${VELOXITY_COLCON_LOG_BASE:-${VELOXITY_WORKSPACE}/log}"
SHIM_PACKAGE_PATH="${VELOXITY_SHIM_PACKAGE_PATH:-${VELOXITY_ROOT}/sim/ros2/veloxity_sil_board_shim}"

if [[ ! -d "${SHIM_PACKAGE_PATH}" ]]; then
  print -u2 "Veloxity ROS 2 shim package not found: ${SHIM_PACKAGE_PATH}"
  if [[ "${_veloxity_sourced}" == "true" ]]; then return 1; else exit 1; fi
fi

if ! command -v cargo >/dev/null 2>&1; then
  print -u2 "cargo is required to build the Veloxity simulator static library."
  if [[ "${_veloxity_sourced}" == "true" ]]; then return 1; else exit 1; fi
fi

if ! command -v colcon >/dev/null 2>&1; then
  print -u2 "colcon is required to build veloxity_sil_board_shim."
  if [[ "${_veloxity_sourced}" == "true" ]]; then return 1; else exit 1; fi
fi

print -P "%F{cyan}building Veloxity simulator static library%f"
(
  cd "${VELOXITY_ROOT}"
  cargo xtask build-sim-lib
)

print -P "%F{cyan}building ROS 2 shim package%f"
(
  cd "${VELOXITY_WORKSPACE}"
  colcon --log-base "${COLCON_LOG_BASE}" build \
    --base-paths "${SHIM_PACKAGE_PATH}" \
    --build-base "${COLCON_BUILD_BASE}" \
    --install-base "${COLCON_INSTALL_BASE}" \
    --packages-select veloxity_sil_board_shim
)

if [[ ! -f "${COLCON_INSTALL_BASE}/setup.zsh" ]]; then
  print -u2 "colcon install overlay was not created: ${COLCON_INSTALL_BASE}/setup.zsh"
  if [[ "${_veloxity_sourced}" == "true" ]]; then return 1; else exit 1; fi
fi

_veloxity_restore_nounset=false
if [[ -o nounset ]]; then
  _veloxity_restore_nounset=true
fi
set +u
source "${COLCON_INSTALL_BASE}/setup.zsh"
if [[ "${_veloxity_restore_nounset}" == "true" ]]; then
  set -u
fi

print -P "%F{green}Veloxity ROS 2 shim built and sourced.%f"
print "overlay: ${COLCON_INSTALL_BASE}/setup.zsh"

if [[ "${_veloxity_sourced}" != "true" ]]; then
  print -u2 "Note: run this with 'source' to keep the overlay in your current shell:"
  print -u2 "  source ${(%):-%x}"
fi

if [[ "${_veloxity_sourced}" == "true" ]]; then
  return 0
fi
