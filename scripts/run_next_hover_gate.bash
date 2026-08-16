#!/usr/bin/env bash
# Run the current 30-second guarded hover gate and print the final diagnosis.

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${_PX4_MPC_ROOT}" || exit 1
source scripts/source_ros2_nmpc.bash || exit 1

# ROS 2 Jazzy setup reads optional variables before defining them, so nounset
# must only be enabled after all environment setup files have been sourced.
set -u

echo "Pre-flight NMPC status:"
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'

echo
echo "Requesting the guarded hover handover now..."
ros2 service call /standard_vtol_nmpc/enable_hover_offboard \
  std_srvs/srv/Trigger '{}'

echo
echo "Watching the 30-second gate; keep QGC ready to select Position mode."
sleep 32

echo
echo "Final NMPC status (hover_test_timeout means the full 30 s gate passed):"
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'

unset _PX4_MPC_ROOT
