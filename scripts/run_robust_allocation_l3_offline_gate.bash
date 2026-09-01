#!/usr/bin/env bash
# Offline prerequisite for live L3a: 10.5 m/s, lambda >= 0.35 live layer.

set -e
_L3_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_L3_ROOT}"
source /opt/ros/jazzy/setup.bash
source "${_L3_ROOT}/install/setup.bash"
source "${_L3_ROOT}/scripts/setup_standard_vtol_nmpc.bash"

PYTHONPATH="${_L3_ROOT}/px4_mpc:${_L3_ROOT}/tools:${_L3_ROOT}/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}" \
    /usr/bin/python3 \
    tools/simulate_standard_vtol_l2_live_layer.py \
    --level 3 --target-speed 10.5 --acceleration 0.30 \
    --minimum-lambda 0.35 --pusher-max 0.42
