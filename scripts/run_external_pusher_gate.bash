#!/usr/bin/env bash
# Run the first guarded NMPC -> custom PX4 pusher pulse.

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_PX4_MPC_ROOT}" || exit 1
source scripts/source_ros2_nmpc.bash || exit 1
set -u

echo "Pre-flight NMPC status:"
if ! timeout 8s ros2 service call /standard_vtol_nmpc/status \
  std_srvs/srv/Trigger '{}'; then
  echo "NMPC service is unavailable. No Offboard command was sent."
  unset _PX4_MPC_ROOT
  exit 1
fi

echo
echo "Requesting guarded external pusher 0 -> 0.05 -> 0 pulse..."
_ENABLE_RESPONSE="$(
  timeout 8s ros2 service call /standard_vtol_nmpc/enable_external_pusher_test \
    std_srvs/srv/Trigger '{}'
)"
printf '%s\n' "${_ENABLE_RESPONSE}"
if [[ "${_ENABLE_RESPONSE}" != *"success=True"* ]]; then
  echo "Gate was not started; fix the reported readiness/configuration reason."
  unset _ENABLE_RESPONSE _PX4_MPC_ROOT
  exit 1
fi

echo
echo "Watching 12 seconds; keep QGC ready to select Position mode."
sleep 14

echo
echo "Final status (external_pusher_test_timeout is the expected flight result):"
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'

unset _ENABLE_RESPONSE _PX4_MPC_ROOT
