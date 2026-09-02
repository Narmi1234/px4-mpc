#!/usr/bin/env bash
# L3b: MC-only, 0->11->0 m/s, lambda >= 0.30, Position fallback.

set -e
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_ROOT}"
source "${_ROOT}/scripts/source_ros2_nmpc.bash"

wait_for_service() {
    local target="$1"
    local attempt discovered
    for attempt in 1 2 3 4 5 6 7 8 9 10; do
        discovered="$(ros2 service list --no-daemon --spin-time 2 2>/dev/null || true)"
        if grep -qx "${target}" <<< "${discovered}"; then return 0; fi
        echo "Waiting for ROS discovery (${attempt}/10)..."
    done
    return 1
}

echo "Checking guarded L3b allocation node..."
if ! wait_for_service "/standard_vtol_robust_shadow/enable_allocation_l3"; then
    echo "ROBUST_ALLOCATION_L3B=FAIL:node_unavailable"
    echo "Start Terminal 3 with standard_vtol_robust_l3b_gate_launch.py."
    exit 1
fi

preflight="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${preflight}"
if [[ "${preflight}" != *"publishes_fmu=True"* \
      || "${preflight}" != *"armed=True"* \
      || "${preflight}" != *"vtol_state=3"* \
      || "${preflight}" != *"solver_failures=0"* \
      || "${preflight}" != *"l3_profile=[speed=11.0,accel=0.25,brake=0.40,lambda=0.30,pusher=0.42]"* ]]; then
    echo "ROBUST_ALLOCATION_L3B=FAIL:wrong_node_or_preflight"
    exit 1
fi

echo "L3b commands about 82 s Offboard: 0->11->0 m/s, lambda 1->0.30->1."
echo "No VTOL transition command is sent; keep QGC ready for Position recovery."
read -r -p "Type YES only while stable at 15-20 m in Position mode: " answer
if [[ "${answer}" != "YES" ]]; then
    echo "ROBUST_ALLOCATION_L3B=CANCELLED"
    exit 1
fi

capture="$(ros2 service call /standard_vtol_robust_shadow/capture_hover_reference std_srvs/srv/Trigger '{}')"
echo "${capture}"
if [[ "${capture}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L3B=FAIL:capture"
    exit 1
fi

echo "Warming the 20 Hz NMPC for eight seconds..."
sleep 8
warm="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${warm}"
if [[ "${warm}" != *"success=True"* || "${warm}" != *"warmup_remaining=0"* ]]; then
    echo "ROBUST_ALLOCATION_L3B=FAIL:warmup"
    exit 1
fi

start="$(ros2 service call /standard_vtol_robust_shadow/enable_allocation_l3 std_srvs/srv/Trigger '{}')"
echo "${start}"
if [[ "${start}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L3B=FAIL:start"
    exit 1
fi

echo "Watching L3b. Do not command a VTOL transition."
for sample in $(seq 1 19); do
    sleep 5
    status="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
    message="$(grep -o "message='[^']*" <<< "${status}" || true)"
    echo "  t=$((sample * 5))s ${message}"
    if [[ "${status}" == *"output_requested=False"* ]]; then break; fi
done

final_status="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "Final status:"
echo "${final_status}"
if [[ "${final_status}" == *"output_requested=False"* \
      && "${final_status}" == *"offboard=False"* \
      && "${final_status}" == *"solver_failures=0"* \
      && "${final_status}" == *"abort_reason=allocation_l3_test_timeout"* \
      && "${final_status}" == *"ever_active=True"* \
      && "${final_status}" == *"ever_valid=True"* \
      && "${final_status}" == *"elevator_ff=0.250"* \
      && "${final_status}" == *"min_lambda=0.3"* ]]; then
    echo "ROBUST_ALLOCATION_L3B=PASS"
    echo "Land, disarm and preserve the ULog. L4 remains locked pending analysis."
    exit 0
fi

echo "ROBUST_ALLOCATION_L3B=FAIL"
echo "Stay in Position, land and do not repeat before reviewing status and ULog."
exit 1
