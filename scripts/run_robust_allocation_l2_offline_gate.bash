#!/usr/bin/env bash
# Offline prerequisite for L2: 9 m/s, lambda >= 0.5, pusher <= 0.35.

set -e
_L2_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_L2_ROOT}"
source "${_L2_ROOT}/.venv/bin/activate"
source "${_L2_ROOT}/scripts/setup_standard_vtol_nmpc.bash"
export PYTHONPATH="${_L2_ROOT}/px4_mpc:${PYTHONPATH}"

python tools/simulate_standard_vtol_robust_transition.py \
    --horizon-steps 20 \
    --target-speed 9.0 \
    --acceleration 0.4 \
    --minimum-lambda 0.5 \
    --pusher-max 0.35 \
    --duration 32.0 \
    --progress \
    --output results/standard_vtol_robust_transition/allocation_l2

echo "ROBUST_ALLOCATION_L2_OFFLINE=PASS"

source /opt/ros/jazzy/setup.bash
source "${_L2_ROOT}/install/setup.bash"
PYTHONPATH="${_L2_ROOT}/px4_mpc:${_L2_ROOT}/.venv/lib/python3.12/site-packages:${PYTHONPATH}" \
    /usr/bin/python3 tools/simulate_standard_vtol_l2_live_layer.py
