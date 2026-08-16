#!/usr/bin/env bash
# Run one guarded 2 m/s multicopter forward-and-stop gate.

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_PX4_MPC_ROOT}" || exit 1
source scripts/source_ros2_nmpc.bash || exit 1
set -u

echo "Pre-flight NMPC status:"
if ! timeout 8s ros2 service call /standard_vtol_nmpc/status \
  std_srvs/srv/Trigger '{}'; then
  echo
  echo "NMPC status service is unavailable. No Offboard command was sent."
  echo "Keep Terminal 3 running and wait for:"
  echo "  Standard VTOL NMPC started in armed-capable guarded MC mode"
  echo "Then run this gate script again from a separate terminal."
  unset _PX4_MPC_ROOT
  exit 1
fi

echo
echo "Requesting the guarded 2 m/s MC-forward gate now..."
_ENABLE_RESPONSE="$(
  timeout 8s ros2 service call /standard_vtol_nmpc/enable_mc_forward_test \
    std_srvs/srv/Trigger '{}'
)"
printf '%s\n' "${_ENABLE_RESPONSE}"
if [[ "${_ENABLE_RESPONSE}" != *"success=True"* ]]; then
  echo "Gate was not started; fix the reported readiness/configuration reason."
  unset _ENABLE_RESPONSE _PX4_MPC_ROOT
  exit 1
fi

echo
echo "The aircraft must remain multicopter; pusher is locked at zero."
echo "Watching the 15-second gate; keep QGC ready to select Position mode."
sleep 17

echo
echo "Final status (mc_forward_test_timeout means every gate criterion passed):"
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'

unset _PX4_MPC_ROOT
unset _ENABLE_RESPONSE
