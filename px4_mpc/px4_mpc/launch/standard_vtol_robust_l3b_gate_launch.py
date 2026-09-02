"""Launch guarded L3b 11 m/s, lambda >= 0.30 allocation gate."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="px4_mpc",
                executable="standard_vtol_robust_shadow",
                name="standard_vtol_robust_shadow",
                output="screen",
                parameters=[
                    {
                        "horizon_steps": 20,
                        "horizon_seconds": 2.0,
                        "active_state_stale_abort_seconds": 0.45,
                        "max_airspeed_age_seconds": 0.75,
                        "allocation_inactive_abort_seconds": 0.35,
                        "allow_l3_output": True,
                        "l3_target_speed": 11.0,
                        "l3_acceleration": 0.25,
                        "l3_brake_rate": 0.40,
                        "l3_hold_seconds": 4.0,
                        "l3_min_lambda": 0.30,
                        "l3_pusher_max": 0.42,
                    }
                ],
            )
        ]
    )
