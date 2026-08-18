#!/usr/bin/env bash
# Run the first guarded NMPC -> custom PX4 pusher pulse.

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_PX4_MPC_ROOT}" || exit 1
source scripts/source_ros2_nmpc.bash || exit 1
set -u

_NMPC_STATUS_SERVICE="/standard_vtol_nmpc/status"
_NMPC_ENABLE_SERVICE="/standard_vtol_nmpc/enable_external_pusher_test"

_service_is_listed() {
  timeout --kill-after=1s 2s ros2 service list 2>/dev/null | grep -Fxq "$1"
}

echo "Checking that the NMPC node from Terminal 3 is running..."
_NMPC_READY=false
for _ATTEMPT in 1 2 3; do
  if _service_is_listed "${_NMPC_STATUS_SERVICE}" && \
     _service_is_listed "${_NMPC_ENABLE_SERVICE}"; then
    _NMPC_READY=true
    break
  fi
  sleep 1
done

if [[ "${_NMPC_READY}" != true ]]; then
  cat <<'EOF'
NMPC node is not available. No Offboard or pusher command was sent.

Start and KEEP OPEN Terminal 3:
  cd /home/imran/Repositories/px4-mpc
  source scripts/source_ros2_nmpc.bash
  ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
    allow_offboard_output:=true \
    allow_external_pusher_output:=true \
    external_pusher_test_max_seconds:=12.0 \
    external_pusher_peak:=0.05 \
    external_pusher_slew:=0.02 \
    external_pusher_hold_seconds:=2.0 \
    external_pusher_start_delay_seconds:=2.0

Wait for the "guarded MC plus guarded external pusher mode" message, then
run this gate again from a separate Terminal 4.
EOF
  unset _ATTEMPT _NMPC_READY _NMPC_STATUS_SERVICE _NMPC_ENABLE_SERVICE \
    _PX4_MPC_ROOT
  exit 1
fi

echo "Pre-flight NMPC status:"
if ! ros2 service call "${_NMPC_STATUS_SERVICE}" std_srvs/srv/Trigger '{}'; then
  echo "NMPC node disappeared before the status check. No Offboard command was sent."
  unset _ATTEMPT _NMPC_READY _NMPC_STATUS_SERVICE _NMPC_ENABLE_SERVICE \
    _PX4_MPC_ROOT
  exit 1
fi

echo
echo "Requesting guarded external pusher 0 -> 0.05 -> 0 pulse..."
_ENABLE_RESPONSE="$(
  ros2 service call "${_NMPC_ENABLE_SERVICE}" \
    std_srvs/srv/Trigger '{}'
)"
printf '%s\n' "${_ENABLE_RESPONSE}"
if [[ "${_ENABLE_RESPONSE}" != *"success=True"* ]]; then
  echo "Gate was not started; fix the reported readiness/configuration reason."
  unset _ENABLE_RESPONSE _ATTEMPT _NMPC_READY _NMPC_STATUS_SERVICE \
    _NMPC_ENABLE_SERVICE _PX4_MPC_ROOT
  exit 1
fi

echo
echo "Watching 12 seconds; keep QGC ready to select Position mode."
sleep 14

echo
echo "Final status (external_pusher_test_timeout is the expected flight result):"
ros2 service call "${_NMPC_STATUS_SERVICE}" std_srvs/srv/Trigger '{}'

unset _ENABLE_RESPONSE _ATTEMPT _NMPC_READY _NMPC_STATUS_SERVICE \
  _NMPC_ENABLE_SERVICE _PX4_MPC_ROOT
