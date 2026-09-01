#!/usr/bin/env bash
# One source command for ROS 2, this workspace, acados and Micro XRCE Agent.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Run this script with: source scripts/source_ros2_nmpc.bash"
    exit 1
fi

_PX4_MPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "Missing /opt/ros/jazzy/setup.bash"
    unset _PX4_MPC_ROOT
    return 1
fi
if [[ ! -f "${_PX4_MPC_ROOT}/install/setup.bash" ]]; then
    echo "Workspace is not built. Run colcon build first."
    unset _PX4_MPC_ROOT
    return 1
fi

source /opt/ros/jazzy/setup.bash
source "${_PX4_MPC_ROOT}/install/setup.bash"
source "${_PX4_MPC_ROOT}/scripts/setup_standard_vtol_nmpc.bash"

# Every PX4-MPC terminal must participate in the same deterministic local DDS
# domain. Shells opened from different IDEs previously inherited different
# ROS_DOMAIN_ID/discovery settings, so a healthy node in Terminal 3 was
# intermittently invisible to Terminal 4. Override the domain only through the
# project-specific variable when simultaneous independent simulations are
# intentionally required.
export ROS_DOMAIN_ID="${PX4_MPC_ROS_DOMAIN_ID:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"
unset ROS_LOCALHOST_ONLY
export PX4_MPC_ROS_ENV="domain=${ROS_DOMAIN_ID},discovery=${ROS_AUTOMATIC_DISCOVERY_RANGE}"
unset _PX4_MPC_ROOT
