#!/usr/bin/env bash
# Run Gate D: one guarded NMPC-coordinated front/back VTOL transition.

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_PX4_MPC_ROOT}" || exit 1
source scripts/source_ros2_nmpc.bash || exit 1
set -u

_STATUS_SERVICE="/standard_vtol_nmpc/status"
_ENABLE_SERVICE="/standard_vtol_nmpc/enable_transition_gate_d"
_DISABLE_SERVICE="/standard_vtol_nmpc/disable"

echo "Checking that the Gate D NMPC node from Terminal 3 is running..."
_SERVICES="$(timeout --kill-after=1s 8s ros2 service list --no-daemon --spin-time 5 2>/dev/null || true)"
if ! grep -Fxq "${_STATUS_SERVICE}" <<<"${_SERVICES}" || \
   ! grep -Fxq "${_ENABLE_SERVICE}" <<<"${_SERVICES}"; then
  echo "Gate D node is unavailable. No command was sent."
  echo "Start Terminal 3 exactly as documented in STANDARD_VTOL_GATE_D_RUNBOOK.md."
  exit 1
fi

echo "Pre-flight NMPC status:"
_PREFLIGHT="$(ros2 service call "${_STATUS_SERVICE}" std_srvs/srv/Trigger '{}')"
printf '%s\n' "${_PREFLIGHT}"
if [[ "${_PREFLIGHT}" != *"gate_d_config=[enabled=True,timeout=90.0,pusher_max=0.300]"* ||
      "${_PREFLIGHT}" != *"nmpc_pusher_max=0.300"* ||
      "${_PREFLIGHT}" != *"gate_d=[state=idle,vtol_state=3"* ||
      "${_PREFLIGHT}" != *"available=True"* ||
      "${_PREFLIGHT}" != *"solver_failures=0"* ||
      "${_PREFLIGHT}" != *"total_solver_failures=0"* ]]; then
  echo "Gate D configuration, MC state, solver, or airspeed stream is not ready."
  echo "No Offboard, pusher, or transition command was sent."
  exit 1
fi

if [[ "${PX4_GATE_D_CONFIRMED:-}" != "YES" ]]; then
  echo
  echo "In the live PX4 shell run and verify:"
  echo "  param set VT_EXT_PUSH_EN 1"
  echo "  param set VT_EXT_PUSH_MAX 0.30"
  echo "  param set VT_EXT_PUSH_SLEW 0.10"
  echo "  param show VT_EXT_PUSH_EN"
  echo "  param show VT_EXT_PUSH_MAX"
  echo "  param show VT_EXT_PUSH_SLEW"
  echo
  echo "Aircraft must be armed in MC Position hover at >=30 m, stable for 10 s,"
  echo "pointed toward >=500 m clear space. QGC must be ready for manual recovery."
  read -r -p "Type exactly GATE-D-READY after checking all items: " _CONFIRM
  if [[ "${_CONFIRM}" != "GATE-D-READY" ]]; then
    echo "Gate D cancelled. No command was sent."
    exit 1
  fi
fi

echo
echo "Requesting Gate D: NMPC 0->8, PX4 front transition, FW hold, back transition, MC brake..."
_ENABLE="$(ros2 service call "${_ENABLE_SERVICE}" std_srvs/srv/Trigger '{}')"
printf '%s\n' "${_ENABLE}"
if [[ "${_ENABLE}" != *"success=True"* ]]; then
  echo "Gate D did not start; fix the readiness reason."
  exit 1
fi

echo
echo "Watching the guarded 90 s PX4-time sequence."
echo "Do not command a QGC transition unless recovery is required."
_FINAL=""
for _POLL in $(seq 1 130); do
  sleep 1
  _STATUS="$(timeout --kill-after=1s 4s ros2 service call \
    "${_STATUS_SERVICE}" std_srvs/srv/Trigger '{}' 2>/dev/null || true)"
  if [[ "${_STATUS}" == *"output_requested=False"* && \
        "${_STATUS}" != *"abort_reason=none"* ]]; then
    _FINAL="${_STATUS}"
    break
  fi
  if (( _POLL % 3 == 0 )); then
    _GATE_STATE="$(printf '%s\n' "${_STATUS}" | sed -n 's/.*gate_d=\[state=\([^,]*\).*/\1/p')"
    _VTOL_STATE="$(printf '%s\n' "${_STATUS}" | sed -n 's/.*vtol_state=\([^,]*\).*/\1/p')"
    _LIFT_WEIGHT="$(printf '%s\n' "${_STATUS}" | sed -n 's/.*gate_d=\[[^]]*lift_weight=\([^,]*\).*/\1/p')"
    _COLLECTIVE="$(printf '%s\n' "${_STATUS}" | sed -n 's/.*, control=\[\([^,]*\).*/\1/p')"
    _PX4_ELAPSED="$(printf '%s\n' "${_STATUS}" | sed -n 's/.*px4_elapsed=\([^,]*\).*/\1/p')"
    echo "  gate=${_GATE_STATE:-unknown}, vtol=${_VTOL_STATE:-unknown}, lift_weight=${_LIFT_WEIGHT:-unknown}, collective=${_COLLECTIVE:-unknown}, px4_elapsed=${_PX4_ELAPSED:-unknown}"
  fi
done

if [[ -z "${_FINAL}" ]]; then
  echo "No final result within 130 wall seconds; requesting Gate D recovery."
  ros2 service call "${_DISABLE_SERVICE}" std_srvs/srv/Trigger '{}' || true
  exit 1
fi

echo
echo "Final status:"
printf '%s\n' "${_FINAL}"
if [[ "${_FINAL}" == *"abort_reason=gate_d_complete"* && \
      "${_FINAL}" == *"gate_d=[state=complete,vtol_state=3"* && \
      "${_FINAL}" == *"front_ack=True,back_ack=True"* && \
      "${_FINAL}" == *"solver_failures=0"* && \
      "${_FINAL}" == *"total_solver_failures=0"* ]]; then
  echo
  echo "ROS_GATE_D=PASS"
  echo "Land, disarm and stop PX4/Gazebo. Then analyze the newest ULog."
  exit 0
fi

echo
echo "ROS_GATE_D=FAIL"
echo "Do not repeat before reviewing abort_reason, gate_d state and the ULog."
exit 1
