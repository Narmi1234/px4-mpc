#!/usr/bin/env bash
# First live NMPC allocation transfer: MC-only, 0->5->0 m/s, lambda >= 0.8.

set -e

_L1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_L1_ROOT}"
source "${_L1_ROOT}/scripts/source_ros2_nmpc.bash"

wait_for_service() {
    local target="$1"
    local attempt
    local discovered
    for attempt in 1 2 3 4 5 6 7 8 9 10; do
        discovered="$(ros2 service list --no-daemon --spin-time 2 2>/dev/null || true)"
        if grep -qx "${target}" <<< "${discovered}"; then
            return 0
        fi
        echo "Waiting for ROS discovery (${attempt}/10)..."
    done
    return 1
}

echo "Checking the guarded L1 allocation node..."
if ! wait_for_service "/standard_vtol_robust_shadow/enable_allocation_l1"; then
    echo "ROBUST_ALLOCATION_L1=FAIL:node_unavailable"
    echo "Start Terminal 3 with standard_vtol_robust_l1_gate_launch.py."
    exit 1
fi

preflight="$(ros2 service call \
    /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${preflight}"
if [[ "${preflight}" != *"publishes_fmu=True"* \
      || "${preflight}" != *"armed=True"* \
      || "${preflight}" != *"vtol_state=3"* \
      || "${preflight}" != *"solver_failures=0"* ]]; then
    echo "ROBUST_ALLOCATION_L1=FAIL:preflight"
    exit 1
fi

echo "This commands about 30 s Offboard: 0->5->0 m/s and lambda 1->0.8->1."
echo "The aircraft must remain MC. QGC Position is the manual recovery."
echo "PX4 must show VT_EXT_PUSH_EN=1, VT_EXT_PUSH_MAX=0.25,"
echo "VT_EXT_ALLOC_EN=1 and VT_EXT_AL_SLEW=0.10."
read -r -p "Type YES only while stable at 8-10 m in Position mode: " answer
if [[ "${answer}" != "YES" ]]; then
    echo "ROBUST_ALLOCATION_L1=CANCELLED"
    exit 1
fi

capture="$(ros2 service call \
    /standard_vtol_robust_shadow/capture_hover_reference \
    std_srvs/srv/Trigger '{}')"
echo "${capture}"
if [[ "${capture}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L1=FAIL:capture"
    exit 1
fi

echo "Warming the 20 Hz NMPC for eight seconds..."
sleep 8
warm="$(ros2 service call \
    /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${warm}"
if [[ "${warm}" != *"success=True"* \
      || "${warm}" != *"warmup_remaining=0"* ]]; then
    echo "ROBUST_ALLOCATION_L1=FAIL:warmup"
    exit 1
fi

start="$(ros2 service call \
    /standard_vtol_robust_shadow/enable_allocation_l1 \
    std_srvs/srv/Trigger '{}')"
echo "${start}"
if [[ "${start}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L1=FAIL:start"
    exit 1
fi

echo "Watching L1. Do not command a VTOL transition."
for sample in 1 2 3 4 5 6 7; do
    sleep 5
    status="$(ros2 service call \
        /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
    message="$(grep -o "message='[^']*" <<< "${status}" || true)"
    echo "  t=$((sample * 5))s ${message}"
    if [[ "${status}" == *"output_requested=False"* ]]; then
        break
    fi
done

final_status="$(ros2 service call \
    /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "Final status:"
echo "${final_status}"

if [[ "${final_status}" == *"output_requested=False"* \
      && "${final_status}" == *"offboard=False"* \
      && "${final_status}" == *"solver_failures=0"* \
      && "${final_status}" == *"abort_reason=allocation_l1_test_timeout"* \
      && "${final_status}" == *"ever_active=True"* \
      && "${final_status}" == *"ever_valid=True"* \
      && "${final_status}" == *"min_lambda=0.8"* ]]; then
    echo "ROBUST_ALLOCATION_L1=PASS"
    echo "Land, disarm and preserve the ULog before attempting a deeper transfer."
    exit 0
fi

echo "ROBUST_ALLOCATION_L1=FAIL"
echo "Stay in Position, land and do not repeat before reviewing status and ULog."
exit 1
