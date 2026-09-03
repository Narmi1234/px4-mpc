#!/usr/bin/env bash
# Keep the ignored px4_msgs dependency aligned with the custom PX4 branch.

set -euo pipefail
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_PX4_ROOT="${PX4_AUTOPILOT_DIR:-/home/imran/Repositories/PX4-Autopilot}"

for message in VtolNmpcAllocationSetpoint VtolNmpcAllocationStatus; do
    source_file="${_PX4_ROOT}/msg/${message}.msg"
    target_file="${_ROOT}/px4_msgs/msg/${message}.msg"
    if [[ ! -f "${source_file}" || ! -d "${_ROOT}/px4_msgs/msg" ]]; then
        echo "Missing PX4 source or px4_msgs dependency for ${message}."
        exit 1
    fi
    cp "${source_file}" "${target_file}"
done

echo "Synced L4 PX4 allocation messages into px4_msgs."
