#!/usr/bin/env bash
# Exact live-layer prerequisite for L3c: 12 m/s, lambda >= 0.20.

set -e
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_ROOT}"
source /opt/ros/jazzy/setup.bash
source "${_ROOT}/install/setup.bash"
source "${_ROOT}/scripts/setup_standard_vtol_nmpc.bash"

PYTHONPATH="${_ROOT}/px4_mpc:${_ROOT}/tools:${_ROOT}/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}" \
    /usr/bin/python3 tools/simulate_standard_vtol_l2_live_layer.py \
    --level 3 --target-speed 12.0 --acceleration 0.25 \
    --brake-rate 0.30 --recovery-seconds 0.1 \
    --brake-entry-lambda 0.70 --minimum-lambda 0.20 \
    --pusher-max 0.45 --collective-min 0.40 --pitch-rate-limit 0.25 \
    --vertical-correction-gain 1.50
