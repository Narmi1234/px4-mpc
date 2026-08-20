#!/usr/bin/env bash
# Run Gate A: one guarded 3 m/s MC pusher-feedback test.

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_PX4_MPC_ROOT}" || exit 1
source scripts/source_ros2_nmpc.bash || exit 1
set -u

_STATUS_SERVICE="/standard_vtol_nmpc/status"
_ENABLE_SERVICE="/standard_vtol_nmpc/enable_pusher_forward_test"
_DISABLE_SERVICE="/standard_vtol_nmpc/disable"

_service_is_listed() {
  timeout --kill-after=1s 2s ros2 service list 2>/dev/null | grep -Fxq "$1"
}

echo "Checking that the Gate A NMPC node from Terminal 3 is running..."
_READY=false
for _ATTEMPT in 1 2 3; do
  if _service_is_listed "${_STATUS_SERVICE}" && \
     _service_is_listed "${_ENABLE_SERVICE}"; then
    _READY=true
    break
  fi
  sleep 1
done

if [[ "${_READY}" != true ]]; then
  cat <<'EOF'
Gate A NMPC node is unavailable. No Offboard or pusher command was sent.
Start Terminal 3 exactly as documented in STANDARD_VTOL_PUSHER_FORWARD_RUNBOOK.md,
keep it open, wait for its startup message, and rerun this script in Terminal 4.
EOF
  exit 1
fi

echo "Pre-flight NMPC status:"
ros2 service call "${_STATUS_SERVICE}" std_srvs/srv/Trigger '{}'

echo
echo "Requesting Gate A: 3 m/s MC pusher feedback..."
_ENABLE_RESPONSE="$(
  ros2 service call "${_ENABLE_SERVICE}" std_srvs/srv/Trigger '{}'
)"
printf '%s\n' "${_ENABLE_RESPONSE}"
if [[ "${_ENABLE_RESPONSE}" != *"success=True"* ]]; then
  echo "Gate A was not started; fix the reported readiness/configuration reason."
  exit 1
fi

echo
echo "Watching until the 20.5 s PX4-time gate finishes."
echo "Keep QGC ready to select Position mode; the aircraft must remain MC."
_FINAL_RESPONSE=""
for _POLL in $(seq 1 45); do
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
  echo "Gate did not finish within 45 wall seconds; requesting safe disable."
  if _service_is_listed "${_DISABLE_SERVICE}"; then
    timeout --kill-after=1s 4s ros2 service call \
      "${_DISABLE_SERVICE}" std_srvs/srv/Trigger '{}' || true
  else
    echo "NMPC node is gone; PX4's Offboard-loss failsafe must take over."
  fi
  exit 1
fi

echo
echo "Final status:"
printf '%s\n' "${_FINAL_RESPONSE}"

if [[ "${_FINAL_RESPONSE}" == *"abort_reason=pusher_forward_test_timeout"* && \
      "${_FINAL_RESPONSE}" == *"solver_failures=0"* ]]; then
  echo
  echo "ROS_GATE=PASS"
  echo "Land, disarm, stop PX4/Gazebo, then run the ULog analyzer from the runbook."
  exit 0
fi

echo
echo "ROS_GATE=FAIL"
echo "Do not repeat the flight before reviewing abort_reason and the ULog."
exit 1
