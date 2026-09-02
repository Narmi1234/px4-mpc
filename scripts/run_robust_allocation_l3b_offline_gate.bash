#!/usr/bin/env bash
# Offline prerequisite for live L3b: 11 m/s, lambda >= 0.30.

set -e
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_ROOT}"
source /opt/ros/jazzy/setup.bash
source "${_ROOT}/install/setup.bash"
source "${_ROOT}/scripts/setup_standard_vtol_nmpc.bash"

PYTHONPATH="${_ROOT}/px4_mpc:${_ROOT}/tools:${_ROOT}/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}" \
    /usr/bin/python3 tools/simulate_standard_vtol_l2_live_layer.py \
    --level 3 --target-speed 11.0 --acceleration 0.25 \
    --brake-rate 0.40 --minimum-lambda 0.30 --pusher-max 0.42
