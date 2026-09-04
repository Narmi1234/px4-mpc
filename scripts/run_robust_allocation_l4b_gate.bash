#!/usr/bin/env bash
# L4b: proven L4a path plus coordinated-course transfer away from MC yaw.

set -e
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_ROOT}"
source "${_ROOT}/scripts/source_ros2_nmpc.bash"

wait_for_service() {
    local target="$1" attempt discovered
    for attempt in $(seq 1 90); do
        discovered="$(ros2 service list --no-daemon --spin-time 2 2>/dev/null || true)"
        if grep -qx "${target}" <<< "${discovered}"; then return 0; fi
        echo "Waiting for L4b node/solver (${attempt}/90)..."
        sleep 1
    done
    return 1
}

echo "Checking guarded L4b coordinated-course node..."
if ! wait_for_service "/standard_vtol_robust_shadow/enable_allocation_l4b"; then
    echo "ROBUST_ALLOCATION_L4B=FAIL:node_unavailable"
    echo "Start Terminal 3 with standard_vtol_robust_l4b_gate_launch.py."
    exit 1
fi

preflight="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${preflight}"
if [[ "${preflight}" != *"publishes_fmu=True"* \
      || "${preflight}" != *"armed=True"* \
      || "${preflight}" != *"vtol_state=3"* \
      || "${preflight}" != *"solver_failures=0"* \
      || "${preflight}" != *"lateral_model=[roll_damping=0.500,roll_surface=0.0475,course_gain=0.95]"* \
      || "${preflight}" != *"l4_transfer=[rp_min=0.05,yaw_min=0.05]"* \
      || "${preflight}" != *"mc_rp_applied="* \
      || "${preflight}" != *"mc_yaw_applied="* ]]; then
    echo "ROBUST_ALLOCATION_L4B=FAIL:wrong_node_model_schema_or_preflight"
    exit 1
fi

echo "L4b repeats the accepted 108 s L4a path and transfers direct MC yaw"
echo "toward the identified coordinated bank/course dynamics. At 12 m/s it"
echo "commands one smooth 0 -> 0.75 -> 0 m lane change. No VTOL mode change."
read -r -p "Type YES only while stable at 20-25 m in Position mode: " answer
if [[ "${answer}" != "YES" ]]; then
    echo "ROBUST_ALLOCATION_L4B=CANCELLED"
    exit 1
fi

capture="$(ros2 service call /standard_vtol_robust_shadow/capture_hover_reference std_srvs/srv/Trigger '{}')"
echo "${capture}"
if [[ "${capture}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L4B=FAIL:capture"
    exit 1
fi

echo "Warming the 20 Hz NMPC for eight seconds..."
sleep 8
warm="$(ros2 service call /standard_vtol_robust_shadow/status std_srvs/srv/Trigger '{}')"
echo "${warm}"
if [[ "${warm}" != *"success=True"* || "${warm}" != *"warmup_remaining=0"* ]]; then
    echo "ROBUST_ALLOCATION_L4B=FAIL:warmup"
    exit 1
fi

start="$(ros2 service call /standard_vtol_robust_shadow/enable_allocation_l4b std_srvs/srv/Trigger '{}')"
echo "${start}"
if [[ "${start}" != *"success=True"* ]]; then
    echo "ROBUST_ALLOCATION_L4B=FAIL:start"
    exit 1
fi

echo "Watching L4b. Expect one small S-like lateral correction near full speed."
echo "Do not command a QGC/PX4 VTOL transition."
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

# The commanded floor is 0.05, but PX4's configured allocation slew and the
# finite full-speed dwell mean the applied proof value need only reach 0.13.
# Parse the measured values instead of matching their first decimal digit.
metric_value() {
    local name="$1"
    sed -nE "s/.*${name}=([0-9]+([.][0-9]+)?).*/\\1/p" <<< "${final_status}"
}
at_most() {
    awk -v value="$1" -v limit="$2" \
        'BEGIN { exit !(value != "" && value + 0.0 <= limit + 0.0) }'
}
min_lambda="$(metric_value min_lambda)"
min_mc_rp="$(metric_value min_mc_rp)"
min_mc_yaw="$(metric_value min_mc_yaw)"

if [[ "${final_status}" == *"output_requested=False"* \
      && "${final_status}" == *"offboard=False"* \
      && "${final_status}" == *"solver_failures=0"* \
      && "${final_status}" == *"abort_reason=allocation_l4b_test_timeout"* \
      && "${final_status}" == *"ever_active=True"* \
      && "${final_status}" == *"ever_valid=True"* ]] \
      && at_most "${min_lambda}" 0.30 \
      && at_most "${min_mc_rp}" 0.13 \
      && at_most "${min_mc_yaw}" 0.13; then
    echo "ROBUST_ALLOCATION_L4B=PASS"
    echo "Land, disarm and preserve the ULog. Motor-off L4c is next."
    exit 0
fi

echo "ROBUST_ALLOCATION_L4B=FAIL"
echo "Stay in Position, land and do not repeat before reviewing status and ULog."
exit 1
