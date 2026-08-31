#!/usr/bin/env bash
# Guarded R3b: exactly five seconds of robust-NMPC hover output.

set -e

_R3B_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_R3B_ROOT}"
source "${_R3B_ROOT}/scripts/source_ros2_nmpc.bash"

echo "Checking the guarded robust-hover node..."
if ! ros2 service list | grep -qx \
    "/standard_vtol_robust_shadow/enable_hover_offboard"; then
    echo "R3b node is unavailable. No Offboard command was sent."
    exit 1
fi

preflight="$(ros2 service call \
    /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${preflight}"
if [[ "${preflight}" != *"publishes_fmu=True"* \
      || "${preflight}" != *"armed=True"* \
      || "${preflight}" != *"solver_failures=0"* ]]; then
    echo "ROBUST_HOVER_OUTPUT=FAIL:preflight"
    exit 1
fi

echo "This will command five seconds of Offboard hover, then request Position."
read -r -p "Type YES only while stable at 5-7 m in Position mode: " answer
if [[ "${answer}" != "YES" ]]; then
    echo "ROBUST_HOVER_OUTPUT=CANCELLED"
    exit 1
fi

capture="$(ros2 service call \
    /standard_vtol_robust_shadow/capture_hover_reference \
    std_srvs/srv/Trigger '{}')"
echo "${capture}"
if [[ "${capture}" != *"success=True"* ]]; then
    echo "ROBUST_HOVER_OUTPUT=FAIL:capture"
    exit 1
fi

echo "Warming and measuring the solver for eight seconds before output..."
sleep 8
warm_status="$(ros2 service call \
    /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${warm_status}"
if [[ "${warm_status}" != *"success=True"* \
      || "${warm_status}" != *"warmup_remaining=0"* \
      || "${warm_status}" != *"solver_failures=0"* ]]; then
    echo "ROBUST_HOVER_OUTPUT=FAIL:warmup"
    exit 1
fi

start="$(ros2 service call \
    /standard_vtol_robust_shadow/enable_hover_offboard \
    std_srvs/srv/Trigger '{}')"
echo "${start}"
if [[ "${start}" != *"success=True"* ]]; then
    echo "ROBUST_HOVER_OUTPUT=FAIL:start"
    exit 1
fi

echo "Watching prestream, five-second Offboard hover and Position fallback..."
sleep 9
final_status="$(ros2 service call \
    /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${final_status}"

if [[ "${final_status}" == *"output_requested=False"* \
      && "${final_status}" == *"offboard=False"* \
      && "${final_status}" == *"solver_failures=0"* \
      && "${final_status}" == *"abort_reason=robust_hover_test_timeout"* ]]; then
    echo "ROBUST_HOVER_OUTPUT=PASS"
    exit 0
fi

echo "ROBUST_HOVER_OUTPUT=FAIL:status"
echo "Land in Position mode and do not repeat before reviewing this status."
exit 1
