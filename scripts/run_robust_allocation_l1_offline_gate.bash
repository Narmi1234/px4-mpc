#!/usr/bin/env bash
# Offline prerequisite for live L1: 5 m/s, lambda >= 0.8, pusher <= 0.25.

set -e
_L1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_L1_ROOT}"
source "${_L1_ROOT}/.venv/bin/activate"
source "${_L1_ROOT}/scripts/setup_standard_vtol_nmpc.bash"
export PYTHONPATH="${_L1_ROOT}/px4_mpc:${PYTHONPATH}"

python tools/simulate_standard_vtol_robust_transition.py \
    --horizon-steps 20 \
    --target-speed 5.0 \
    --acceleration 0.4 \
    --minimum-lambda 0.8 \
    --pusher-max 0.25 \
    --duration 22.0 \
    --progress \
    --output results/standard_vtol_robust_transition/allocation_l1

echo "ROBUST_ALLOCATION_L1_OFFLINE=PASS"
