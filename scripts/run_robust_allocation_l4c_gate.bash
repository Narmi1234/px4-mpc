#!/usr/bin/env bash
# L4c: guarded zero lift-motor allocation while PX4 retains MC recovery state.

set -e
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_ROOT}"
source "${_ROOT}/scripts/source_ros2_nmpc.bash"

wait_for_service() {
    local target="$1" attempt discovered
    for attempt in $(seq 1 90); do
        discovered="$(ros2 service list --no-daemon --spin-time 2 2>/dev/null || true)"
        if grep -qx "${target}" <<< "${discovered}"; then return 0; fi
        echo "Waiting for L4c node/solver (${attempt}/90)..."
        sleep 1
    done
    return 1
}

echo "Checking guarded L4c motor-off node..."
if ! wait_for_service "/standard_vtol_robust_shadow/enable_allocation_l4c"; then
    echo "ROBUST_ALLOCATION_L4C=FAIL:node_unavailable"
    echo "Start Terminal 3 with standard_vtol_robust_l4c_gate_launch.py."
    exit 1
fi

preflight="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${preflight}"
if [[ "${preflight}" != *"publishes_fmu=True"* \
      || "${preflight}" != *"armed=True"* \
      || "${preflight}" != *"vtol_state=3"* \
      || "${preflight}" != *"solver_failures=0"* \
      || "${preflight}" != *"l3_profile=[speed=12.0,accel=0.25,brake=0.30,lambda=0.00,pusher=0.45]"* \
      || "${preflight}" != *"l4_transfer=[rp_min=0.05,yaw_min=0.05]"* ]]; then
    echo "ROBUST_ALLOCATION_L4C=FAIL:wrong_node_model_or_preflight"
    exit 1
fi

echo "DANGER: L4c commands lift allocation to zero at 12 m/s for >=2 s."
echo "PX4 remains in MC state only so Position fallback can restore the motors."
echo "Do not command a QGC/PX4 VTOL transition. Keep Position ready."
read -r -p "Type MOTOR-OFF only while stable at 20-25 m in Position mode: " answer
if [[ "${answer}" != "MOTOR-OFF" ]]; then
    echo "ROBUST_ALLOCATION_L4C=CANCELLED"
    exit 1
fi

capture="$(ros2 service call /standard_vtol_robust_shadow/capture_hover_reference std_srvs/srv/Trigger '{}')"
echo "${capture}"
if [[ "${capture}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L4C=FAIL:capture"
    exit 1
fi

echo "Warming the 20 Hz NMPC for eight seconds..."
sleep 8
warm="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${warm}"
if [[ "${warm}" != *"success=True"* || "${warm}" != *"warmup_remaining=0"* ]]; then
    echo "ROBUST_ALLOCATION_L4C=FAIL:warmup"
    exit 1
fi

start="$(ros2 service call /standard_vtol_robust_shadow/enable_allocation_l4c std_srvs/srv/Trigger '{}')"
echo "${start}"
if [[ "${start}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L4C=FAIL:start"
    exit 1
fi

echo "Watching L4c. The motors should unload gradually, never switch abruptly."
for sample in $(seq 1 25); do
    sleep 5
    status="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
    message="$(grep -o "message='[^']*" <<< "${status}" || true)"
    echo "  t=$((sample * 5))s ${message}"
    if [[ "${status}" == *"output_requested=False"* ]]; then break; fi
done

final_status="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "Final status:"
echo "${final_status}"
metric_value() {
    local name="$1"
    sed -nE "s/.*${name}=([0-9]+([.][0-9]+)?).*/\\1/p" <<< "${final_status}"
}
at_most() {
    awk -v value="$1" -v limit="$2" \
        'BEGIN { exit !(value != "" && value + 0.0 <= limit + 0.0) }'
}
at_least() {
    awk -v value="$1" -v limit="$2" \
        'BEGIN { exit !(value != "" && value + 0.0 >= limit + 0.0) }'
}

if [[ "${final_status}" == *"output_requested=False"* \
      && "${final_status}" == *"offboard=False"* \
      && "${final_status}" == *"solver_failures=0"* \
      && "${final_status}" == *"abort_reason=allocation_l4c_test_timeout"* \
      && "${final_status}" == *"ever_active=True"* \
      && "${final_status}" == *"ever_valid=True"* ]] \
      && at_most "$(metric_value min_lambda)" 0.03 \
      && at_most "$(metric_value min_mc_rp)" 0.13 \
      && at_most "$(metric_value min_mc_yaw)" 0.13 \
      && at_least "$(metric_value motor_off_s)" 2.0; then
    echo "ROBUST_ALLOCATION_L4C=PASS"
    echo "Land, disarm and preserve the ULog. Full VTOL-state transition is next."
    exit 0
fi

echo "ROBUST_ALLOCATION_L4C=FAIL"
echo "Stay in Position, land and do not repeat before reviewing status and ULog."
exit 1
