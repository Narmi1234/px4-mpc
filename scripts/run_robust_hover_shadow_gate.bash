#!/usr/bin/env bash
# Read-only R3a gate. This script never calls an Offboard/output service.

set -e

_SHADOW_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_SHADOW_ROOT}"
source "${_SHADOW_ROOT}/scripts/source_ros2_nmpc.bash"

echo "Checking the read-only robust shadow node..."
if ! ros2 service list | grep -qx "/standard_vtol_robust_shadow/status"; then
    echo "R3a shadow node is unavailable. No command was sent."
    echo "Start Terminal 3 from STANDARD_VTOL_ROBUST_TRANSITION_RUNBOOK.md."
    exit 1
fi

echo "Capturing the current armed Position-mode hover reference..."
capture_output="$(ros2 service call \
    /standard_vtol_robust_shadow/capture_hover_reference \
    std_srvs/srv/Trigger '{}')"
echo "${capture_output}"
if [[ "${capture_output}" != *"success=True"* ]]; then
    echo "ROBUST_HOVER_SHADOW=FAIL:capture"
    exit 1
fi

echo "Watching ten read-only status samples (15 seconds total)..."
final_status=""
for _index in 1 2 3 4 5 6 7 8 9 10; do
    sleep 1.5
    final_status="$(ros2 service call \
        /standard_vtol_robust_shadow/status \
        std_srvs/srv/Trigger '{}')"
    echo "${final_status}"
done

if [[ "${final_status}" == *"success=True"* \
      && "${final_status}" == *"read_only=True"* \
      && "${final_status}" == *"publishes_fmu=False"* \
      && "${final_status}" == *"solver_failures=0"* ]]; then
    echo "ROBUST_HOVER_SHADOW=PASS"
    exit 0
fi

echo "ROBUST_HOVER_SHADOW=FAIL:status"
exit 1
