#!/usr/bin/env bash
# Run the accepted robust front-transition offline matrix. No ROS/PX4 output.

set -e

_ROBUST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_ROBUST_ROOT}"

source "${_ROBUST_ROOT}/.venv/bin/activate"
source "${_ROBUST_ROOT}/scripts/setup_standard_vtol_nmpc.bash"
export PYTHONPATH="${_ROBUST_ROOT}/px4_mpc:${PYTHONPATH}"

run_case() {
    local name="$1"
    shift
    echo
    echo "ROBUST_CASE=${name}"
    python tools/simulate_standard_vtol_robust_transition.py \
        --horizon-steps 25 \
        --output "results/standard_vtol_robust_transition/${name}" "$@"
}

run_case nominal
run_case estimated_plus_0235 --pitch-disturbance 0.235
run_case estimated_minus_0235 --pitch-disturbance -0.235
run_case unmodeled_plus_010 --pitch-disturbance 0.10 --unmodeled-disturbance
run_case unmodeled_minus_010 --pitch-disturbance -0.10 --unmodeled-disturbance

echo
echo "ROBUST_TRANSITION_MATRIX=PASS"
