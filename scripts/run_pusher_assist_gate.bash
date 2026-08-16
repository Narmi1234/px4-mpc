#!/usr/bin/env bash
# Run one guarded 3 m/s PX4 multicopter pusher-assist gate.

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_PX4_MPC_ROOT}" || exit 1
source scripts/source_ros2_nmpc.bash || exit 1
set -u

echo "Pre-flight NMPC/supervisor status:"
if ! timeout 8s ros2 service call /standard_vtol_nmpc/status \
  std_srvs/srv/Trigger '{}'; then
  echo
  echo "Supervisor status service is unavailable. No Offboard command was sent."
  unset _PX4_MPC_ROOT
  exit 1
fi

echo
echo "Requesting guarded 3 m/s PX4 pusher-assist gate now..."
_ENABLE_RESPONSE="$(
  timeout 8s ros2 service call /standard_vtol_nmpc/enable_pusher_assist_test \
    std_srvs/srv/Trigger '{}'
)"
printf '%s\n' "${_ENABLE_RESPONSE}"
if [[ "${_ENABLE_RESPONSE}" != *"success=True"* ]]; then
  echo "Gate was not started; fix the reported readiness/configuration reason."
  unset _ENABLE_RESPONSE _PX4_MPC_ROOT
  exit 1
fi

echo
echo "PX4 owns position/rate/lift and its VTOL pusher-assist law in this gate."
echo "Watching 13 seconds; keep QGC ready to select Position mode."
sleep 15

echo
echo "Final status (pusher_assist_test_timeout means the flight gates passed):"
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'

unset _ENABLE_RESPONSE _PX4_MPC_ROOT
