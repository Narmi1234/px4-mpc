#!/usr/bin/env bash
# Run Gate B1: one guarded 5 m/s MC pre-transition test.

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_PX4_MPC_ROOT}" || exit 1
source scripts/source_ros2_nmpc.bash || exit 1
set -u

_STATUS_SERVICE="/standard_vtol_nmpc/status"
_ENABLE_SERVICE="/standard_vtol_nmpc/enable_pretransition_5mps_test"
_DISABLE_SERVICE="/standard_vtol_nmpc/disable"

echo "Checking that the Gate B1 NMPC node from Terminal 3 is running..."
_READY=false
_DISCOVERED_SERVICES=""
for _ATTEMPT in 1 2 3; do
  _DISCOVERED_SERVICES="$(
    timeout --kill-after=1s 7s ros2 service list \
      --no-daemon --spin-time 4 2>/dev/null || true
  )"
  if grep -Fxq "${_STATUS_SERVICE}" <<<"${_DISCOVERED_SERVICES}" && \
     grep -Fxq "${_ENABLE_SERVICE}" <<<"${_DISCOVERED_SERVICES}"; then
    _READY=true
    break
  fi
  sleep 1
done

if [[ "${_READY}" != true ]]; then
  echo "Gate B1 node is unavailable. No Offboard or pusher command was sent."
  echo "Terminal 4 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}."
  echo "Start Terminal 3 from STANDARD_VTOL_GATE_B_RUNBOOK.md and rerun."
  exit 1
fi

echo "Pre-flight NMPC status:"
_PREFLIGHT_STATUS="$(
  ros2 service call "${_STATUS_SERVICE}" std_srvs/srv/Trigger '{}'
)"
printf '%s\n' "${_PREFLIGHT_STATUS}"
if [[ "${_PREFLIGHT_STATUS}" != *"pretransition_profile=[speed=5.0,accel=0.40,hold=3.0]"* ||
      "${_PREFLIGHT_STATUS}" != *"nmpc_pusher_max=0.150"* ||
      "${_PREFLIGHT_STATUS}" != *"available=True"* ]]; then
  echo "Gate B1 pre-flight configuration or airspeed stream is unavailable."
  echo "No Offboard or pusher command was sent."
  exit 1
fi

if [[ "${PX4_PUSHER_PARAMS_CONFIRMED:-}" != "YES" ]]; then
  echo
  echo "In the live PX4 shell, run and verify:"
  echo "  param set VT_EXT_PUSH_EN 1"
  echo "  param set VT_EXT_PUSH_MAX 0.15"
  echo "  param set VT_EXT_PUSH_SLEW 0.10"
  echo "  param show VT_EXT_PUSH_EN"
  echo "  param show VT_EXT_PUSH_MAX"
  echo "  param show VT_EXT_PUSH_SLEW"
  read -r -p "Type YES only if PX4 displayed 1, 0.15, 0.10: " \
    _PX4_PARAMETER_CONFIRMATION
  if [[ "${_PX4_PARAMETER_CONFIRMATION}" != "YES" ]]; then
    echo "Gate B1 cancelled. No Offboard or pusher command was sent."
    exit 1
  fi
fi

echo
echo "Requesting Gate B1: guarded 5 m/s MC pre-transition profile..."
_ENABLE_RESPONSE="$(
  ros2 service call "${_ENABLE_SERVICE}" std_srvs/srv/Trigger '{}'
)"
printf '%s\n' "${_ENABLE_RESPONSE}"
if [[ "${_ENABLE_RESPONSE}" != *"success=True"* ]]; then
  echo "Gate B1 was not started; fix the reported readiness reason."
  exit 1
fi

echo
echo "Watching until the 49.0 s PX4-time gate finishes."
echo "The aircraft must remain MC; keep QGC ready to select Position mode."
_FINAL_RESPONSE=""
for _POLL in $(seq 1 80); do
  sleep 1
  _STATUS_RESPONSE="$(
    timeout --kill-after=1s 4s ros2 service call \
      "${_STATUS_SERVICE}" std_srvs/srv/Trigger '{}' 2>/dev/null || true
  )"
  if [[ "${_STATUS_RESPONSE}" == *"output_requested=False"* && \
        "${_STATUS_RESPONSE}" != *"abort_reason=none"* ]]; then
    _FINAL_RESPONSE="${_STATUS_RESPONSE}"
    break
  fi
  if (( _POLL % 5 == 0 )); then
    _PHASE="$(printf '%s\n' "${_STATUS_RESPONSE}" | \
      sed -n 's/.*profile_phase=\([^,]*\).*/\1/p')"
    _PX4_ELAPSED="$(printf '%s\n' "${_STATUS_RESPONSE}" | \
      sed -n 's/.*px4_elapsed=\([^,]*\).*/\1/p')"
    echo "  phase=${_PHASE:-unknown}, px4_elapsed=${_PX4_ELAPSED:-unknown}"
  fi
done

if [[ -z "${_FINAL_RESPONSE}" ]]; then
  echo "Gate did not finish within 80 wall seconds; requesting safe disable."
  if timeout --kill-after=1s 7s ros2 service list \
      --no-daemon --spin-time 4 2>/dev/null | \
      grep -Fxq "${_DISABLE_SERVICE}"; then
    timeout --kill-after=1s 4s ros2 service call \
      "${_DISABLE_SERVICE}" std_srvs/srv/Trigger '{}' || true
  fi
  exit 1
fi

echo
echo "Final status:"
printf '%s\n' "${_FINAL_RESPONSE}"

if [[ "${_FINAL_RESPONSE}" == *"abort_reason=pretransition_5mps_test_timeout"* && \
      "${_FINAL_RESPONSE}" == *"solver_failures=0"* ]]; then
  echo
  echo "ROS_GATE=PASS"
  echo "Land, disarm and close PX4/Gazebo before running the B1 ULog analyzer."
  exit 0
fi

echo
echo "ROS_GATE=FAIL"
echo "Do not repeat the flight before reviewing abort_reason and the ULog."
exit 1
