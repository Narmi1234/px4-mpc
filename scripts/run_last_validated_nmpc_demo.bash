#!/usr/bin/env bash
# Stable entry point for the last fully accepted live NMPC demonstration.

set -e
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Last validated Standard VTOL NMPC demo: L4b authority transfer"
echo "This is not a full VTOL transition and does not stop the lift motors."
exec bash "${_ROOT}/scripts/run_robust_allocation_l4b_gate.bash"
