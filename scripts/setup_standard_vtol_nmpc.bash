#!/usr/bin/env bash
# Source this file after ROS 2 and the workspace setup files.

_STANDARD_VTOL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ACADOS_SOURCE_DIR="${_STANDARD_VTOL_ROOT}/acados"
export MICRO_XRCE_AGENT_DIR="${_STANDARD_VTOL_ROOT}/microxrce_agent_install"
export LD_LIBRARY_PATH="${MICRO_XRCE_AGENT_DIR}/lib:${ACADOS_SOURCE_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${_STANDARD_VTOL_ROOT}/.venv/lib/python3.12/site-packages:${ACADOS_SOURCE_DIR}/interfaces/acados_template${PYTHONPATH:+:${PYTHONPATH}}"
export MPLCONFIGDIR="/tmp/matplotlib-px4-mpc"
export ROS_LOG_DIR="/tmp/px4-mpc-ros-logs"
unset _STANDARD_VTOL_ROOT
