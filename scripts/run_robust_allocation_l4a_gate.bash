#!/usr/bin/env bash
# L4a: validated L3c lift path plus independent roll/pitch torque transfer.

set -e
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_ROOT}"
source "${_ROOT}/scripts/source_ros2_nmpc.bash"

wait_for_service() {
    local target="$1" attempt discovered
    for attempt in $(seq 1 90); do
        discovered="$(ros2 service list --no-daemon --spin-time 2 2>/dev/null || true)"
        if grep -qx "${target}" <<< "${discovered}"; then return 0; fi
        echo "Waiting for L4a node/solver (${attempt}/90)..."
        sleep 1
    done
    return 1
}

echo "Checking guarded L4a torque-transfer node..."
if ! wait_for_service "/standard_vtol_robust_shadow/enable_allocation_l4a"; then
    echo "ROBUST_ALLOCATION_L4A=FAIL:node_unavailable"
    echo "Start Terminal 3 with standard_vtol_robust_l4a_gate_launch.py."
    exit 1
fi

preflight="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${preflight}"
if [[ "${preflight}" != *"publishes_fmu=True"* \
      || "${preflight}" != *"armed=True"* \
      || "${preflight}" != *"vtol_state=3"* \
      || "${preflight}" != *"solver_failures=0"* \
      || "${preflight}" != *"lateral_model=[roll_damping=0.500,roll_surface=0.0475,course_gain=0.95]"* \
      || "${preflight}" != *"mc_rp_applied="* \
      || "${preflight}" != *"mc_yaw_applied="* ]]; then
    echo "ROBUST_ALLOCATION_L4A=FAIL:wrong_node_model_schema_or_preflight"
    exit 1
fi

echo "L4a uses the proven 108 s L3c path, but transfers 95% of roll/pitch"
echo "torque to the wing surfaces. It retains MC yaw and never requests FW mode."
read -r -p "Type YES only while stable at 20-25 m in Position mode: " answer
if [[ "${answer}" != "YES" ]]; then
    echo "ROBUST_ALLOCATION_L4A=CANCELLED"
    exit 1
fi

capture="$(ros2 service call /standard_vtol_robust_shadow/capture_hover_reference std_srvs/srv/Trigger '{}')"
echo "${capture}"
if [[ "${capture}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L4A=FAIL:capture"
    exit 1
fi

echo "Warming the 20 Hz NMPC for eight seconds..."
sleep 8
warm="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${warm}"
if [[ "${warm}" != *"success=True"* || "${warm}" != *"warmup_remaining=0"* ]]; then
    echo "ROBUST_ALLOCATION_L4A=FAIL:warmup"
    exit 1
fi

start="$(ros2 service call /standard_vtol_robust_shadow/enable_allocation_l4a std_srvs/srv/Trigger '{}')"
echo "${start}"
if [[ "${start}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L4A=FAIL:start"
    exit 1
fi

echo "Watching L4a. Do not command a QGC/PX4 VTOL transition."
for sample in $(seq 1 23); do
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
      && "${final_status}" == *"abort_reason=allocation_l4a_test_timeout"* \
      && "${final_status}" == *"ever_active=True"* \
      && "${final_status}" == *"ever_valid=True"* \
      && "${final_status}" == *"min_lambda=0.2"* \
      && "${final_status}" == *"min_mc_rp=0.0"* \
      && "${final_status}" == *"min_mc_yaw=1.000"* ]]; then
    echo "ROBUST_ALLOCATION_L4A=PASS"
    echo "Land, disarm and preserve the ULog. L4b coordinated-course is next."
    exit 0
fi

echo "ROBUST_ALLOCATION_L4A=FAIL"
echo "Stay in Position, land and do not repeat before reviewing status and ULog."
exit 1
