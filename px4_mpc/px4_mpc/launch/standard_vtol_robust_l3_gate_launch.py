"""Launch guarded L3a 10.5 m/s, lambda >= 0.35 allocation gate."""

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
                        "allow_l3_output": True,
                        "l3_target_speed": 10.5,
                        "l3_acceleration": 0.30,
                        "l3_hold_seconds": 4.0,
                        "l3_min_lambda": 0.35,
                        "l3_pusher_max": 0.42,
                    }
                ],
            )
        ]
    )
