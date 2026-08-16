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
unset _PX4_MPC_ROOT
